"""Fly the trained MARL policy on top of RAPTOR in Isaac Sim (Pegasus, no PX4): take off, tension, lift, then RL.

Three S500s stand on the ground around the payload with slack cables. The run then goes through the sequence a real
flight will follow, with RAPTOR flying every drone throughout:

  1. take-off      each drone climbs straight up to just below the point where its cable goes taut
  2. tension/lift  all three rise together, slowly: the cables go taut and the payload leaves the ground
  3. hold          the formation holds; the run only goes on if every cable is taut and the payload is airborne
  4. RL            the MARL policy takes over the three position setpoints (50 Hz) and holds the payload where it is
  5. goals         the payload is sent to each goal pose in turn; the goal is drawn as a green disc with axes

Phases 1 to 3 are scripted setpoints, standing in for the operator. From phase 4 on each drone runs its own copy of
the exported policy (policy_falcon<slot + 1>.onnx of a package in models/ at the repo root) through marl_policy.py, which is checked against the
training environment. RAPTOR is the unchanged RaptorBackend of ../raptor.

    source env/env.sh
    cd components/simulation/tests/marl_raptor
    python marl_payload.py --headless                                  # default goal
    python marl_payload.py --headless --goal=0.8,0.3,1.3,0,0,45 --goal=-0.5,0.5,0.9
    python marl_payload.py                                             # window: move the goal with the keyboard

A goal is x,y,z in metres in the world frame, optionally followed by roll,pitch,yaw in degrees. With the window and
no --goal, keys move the goal: I/K x, J/L y, U/O z (0.25 m a press), H back to where the policy took over.

--trained_on says which task the policy in --models comes from (marl_policy.TRAINED_ON). A policy trained with Falcon
drones (the default) calls a point 0.03 m above the cable tie point the drone's position, so on the S500 it is given
that point and RAPTOR's target is shifted back by the same offset. A policy trained on the S500 task is given the
body's centre of mass, on that task's rig, with its step scale and setpoint speed cap:

    python marl_payload.py --headless --trained_on s500

Exits 1 if the cables are not taut at hand-over or a goal is missed. Logs go to runs/.
"""

import argparse
import dataclasses
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CASTOR_ROOT = os.environ.get("CASTOR_ROOT") or os.path.abspath(os.path.join(HERE, "../../../.."))
PEGASUS_EXT = os.path.join(CASTOR_ROOT, "components/simulation/pegasus_simulator/extensions/pegasus.simulator")
ASSETS = os.path.join(CASTOR_ROOT, "components/simulation/assets")
RAPTOR_TESTS = os.path.join(HERE, "../raptor")
DEFAULT_RAPTOR = os.path.join(CASTOR_ROOT, "components/vehicle/PX4-Autopilot/src/modules/mc_raptor/blob/policy.tar")
sys.path.insert(0, ASSETS)

from castor_assets import config as C  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--goal", action="append", default=[], metavar="X,Y,Z[,ROLL,PITCH,YAW]",
                    help="payload goal, world frame, metres and degrees; repeatable, flown in order. "
                         "Write --goal=-0.5,0.5,0.9 when the first number is negative")
parser.add_argument("--goal_time", type=float, default=12.0, help="seconds given to each goal")
parser.add_argument("--lift_height", type=float, default=1.0, help="payload height above the ground after the lift")
parser.add_argument("--takeoff_speed", type=float, default=0.5, help="m/s, mean climb rate of the take-off")
parser.add_argument("--lift_speed", type=float, default=0.15, help="m/s while the cables go taut and the payload lifts")
parser.add_argument("--trained_on", choices=("falcon", "s500"), default="falcon",
                    help="the task the policy in --models was trained on: sets the rig, the point given to the policy "
                         "as the drone's position, the step scale and the setpoint speed cap. See the README")
parser.add_argument("--step_scale", type=float, default=None,
                    help="metres a unit policy action moves a setpoint per 20 ms step. Default: 0.025 for a Falcon "
                         "policy (trained with 0.05, which swings the S500's setpoints at about 2 Hz, see the "
                         "README), training's 0.02 for an S500 policy")
parser.add_argument("--max_speed", type=float, default=None,
                    help="m/s, cap on how fast a policy setpoint moves. Default: none for a Falcon policy, training's "
                         "1.0 for an S500 policy")
parser.add_argument("--no_rl", action="store_true", help="stop after the hold: RAPTOR alone, no policy")
parser.add_argument("--models", default=None, help="model package with policy_falcon<i>.onnx; default: models/DEFAULT")
parser.add_argument("--raptor", default=DEFAULT_RAPTOR, help="RAPTOR checkpoint (policy.tar)")
parser.add_argument("--rig", default=None,
                    help="rig config (path, or a name in assets/config/); default: the one --trained_on was trained on")
parser.add_argument("--vehicle", default=None, help="vehicle config; default: the rig's vehicle_config")
parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="rig override, repeatable")
parser.add_argument("--vset", action="append", default=[], metavar="KEY=VALUE", help="vehicle override, repeatable")
parser.add_argument("--physics_hz", type=float, default=400.0)
parser.add_argument("--render_hz", type=float, default=50.0)
parser.add_argument("--fast", action="store_true", help="GUI: run as fast as possible instead of real time")
parser.add_argument("--keep_open", action="store_true", help="GUI: keep flying after the goals; the keyboard moves the goal")
parser.add_argument("--snapshot", default=None, metavar="PNG", help="headless: save a picture of the scene at the end")
parser.add_argument("--log", default=None)
parser.add_argument("--diag", action="append", default=[], metavar="NAME[=VALUE]",
                    help="diagnostics: no_vel_ff (setpoint velocity sent as zero), filter=A (low-pass the actions, "
                         "0 < A <= 1), point=Z (policy point Z m above the tie point instead of 0.03), "
                         "raptor_100hz (RAPTOR stepped at a plain 100 Hz, as in training)")
args = parser.parse_args()

# marl_policy needs only NumPy: the settings of the task the policy was trained on
sys.path.insert(0, HERE)
import marl_policy as MP  # noqa: E402

args.models = args.models or MP.default_models()
trained = MP.TRAINED_ON[args.trained_on]
if args.rig is None:
    args.rig = trained["rig"]
if args.step_scale is None:
    args.step_scale = 0.025 if args.trained_on == "falcon" else trained["step_scale"]
if args.max_speed is None:
    args.max_speed = trained["max_speed"]

# config mistakes fail here, before Isaac Sim starts
rig = C.load_rig(args.rig, args.set)
vehicle_cfg = C.load_vehicle(args.vehicle or rig.vehicle_config, args.vset)
if rig.vehicle != "s500":
    parser.error("the take-off sequence needs the S500 (its landing gear and cable mount)")


def parse_goal(text):
    v = [float(x) for x in text.split(",")]
    if len(v) not in (3, 6):
        parser.error(f"--goal {text!r}: expected x,y,z or x,y,z,roll,pitch,yaw")
    return v + [0.0] * (6 - len(v))


goals = [parse_goal(g) for g in args.goal]
manual_goal = not goals and not args.headless
if not goals and args.headless:
    goals = [[0.6, 0.4, args.lift_height + 0.2, 0.0, 0.0, 30.0]]
if args.log is None:
    args.log = os.path.join(HERE, "runs", f"marl_{rig.cable.model}_n{rig.num_drones}.npz")

sys.stdout.reconfigure(line_buffering=True)  # SimulationApp.close() hard-exits and would drop buffered output

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("omni.isaac.dynamic_control")  # Pegasus 5.1 still uses it
simulation_app.update()

sys.path.insert(0, PEGASUS_EXT)
sys.path.insert(0, os.path.join(HERE, ".deps"))  # pymavlink (Pegasus imports it) and onnxruntime, see README
sys.path.insert(0, RAPTOR_TESTS)
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import torch  # noqa: E402
from isaacsim.core.api.objects import GroundPlane  # noqa: E402
from isaacsim.core.api.world import World  # noqa: E402
from pxr import UsdLux  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from pegasus.simulator.logic.dynamics import LinearDrag  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.params import ROBOTS  # noqa: E402

from marl_scene import GoalMarker  # noqa: E402
from castor_assets import usd_build as U  # noqa: E402
from castor_assets.runtime import RigRuntime  # noqa: E402
from keyboard_teleop import KeyboardTeleop  # noqa: E402
from raptor_backend import RaptorBackend  # noqa: E402
from raptor_policy import RaptorPolicy  # noqa: E402
from sim_loop import run_physics  # noqa: E402

torch.set_num_threads(1)

DIAG = dict(item.split("=", 1) if "=" in item else (item, "1") for item in args.diag)
TAUT_TOLERANCE = 0.03  # m: a cable within this of its length counts as taut
SLACK_MARGIN = 0.10  # m: the take-off stops this far below the point where the cables go taut
GOAL_BOX = (np.array([-1.0, -1.0, 0.5]), np.array([1.0, 1.0, 1.5]))  # m: the goal positions seen in training


class Raptor100Hz(RaptorBackend):
    """Diagnostic: RAPTOR the way the training environment runs it, one evaluation every 10 ms, output held between."""

    def update(self, dt):
        if self.substep % self.native_every == 0:
            super().update(dt)  # a native step: advances the GRU, substep and t
        else:
            self.substep += 1
            self.t += dt


class LaggedThrustCurve(QuadraticThrustCurve):
    """QuadraticThrustCurve with a first-order motor lag (propulsion.motor_time_constant), as in ../raptor."""

    def __init__(self, config, tau):
        super().__init__(config)
        self.tau = tau

    def update(self, state, dt):
        a = 1.0 - math.exp(-dt / self.tau)
        rolling = 0.0
        for i in range(self._num_rotors):
            ref = min(max(self._input_reference[i], self.min_rotor_velocity[i]), self.max_rotor_velocity[i])
            self._velocity[i] += a * (ref - self._velocity[i])
            self._force[i] = self._rotor_constant[i] * self._velocity[i] ** 2
            rolling += self._rolling_moment_coefficient[i] * self._velocity[i] ** 2 * self._rot_dir[i]
        self._rolling_moment = rolling
        return self._force, self._velocity, self._rolling_moment


def min_jerk(s):
    """Position and velocity shape of a rest-to-rest move, s in [0, 1]."""
    s = min(max(s, 0.0), 1.0)
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5, 30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4


class Sequence:
    """Where every drone's RAPTOR target is, phase by phase, and the policy once it has taken over."""

    def __init__(self, ground, slack, lifted, yaws, point_local, rig_rt, n):
        self.ground, self.slack, self.lifted, self.yaws = ground, slack, lifted, yaws
        self.point_local = point_local  # the policy's drone point, body frame
        self.rig_rt, self.n = rig_rt, n
        t_idle = 1.0
        climb = max(float(np.linalg.norm(slack[0] - ground[0])), 1e-6)
        rise = max(float(np.linalg.norm(lifted[0] - slack[0])), 1e-6)
        # a rest-to-rest move peaks at 1.875 times its mean speed; the speeds given are the mean
        self.t_takeoff = (t_idle, t_idle + climb / args.takeoff_speed)
        self.t_lift = (self.t_takeoff[1] + 2.0, self.t_takeoff[1] + 2.0 + rise / args.lift_speed)
        self.t_rl = self.t_lift[1] + 4.0
        self.t_goals = self.t_rl + 3.0
        self.duration = self.t_rl if args.no_rl else self.t_goals + len(goals) * args.goal_time
        self.phase = "ground"
        self.rl = False
        self.handover_ok = None
        self.goal_pos, self.goal_rot, self.goal_index = None, None, -1
        self.goal_home = None
        self.observations = [MP.Observation(i, n) for i in range(n)]
        self.integrators = [MP.SetpointIntegrator(step_scale=args.step_scale, max_speed=args.max_speed)
                            for _ in range(n)]
        self.policies = None
        self.filtered = [None] * n
        self.backends = None
        self.teleop = None
        self.log = {k: [] for k in ("t", "goal_pos", "goal_rot", "payload_pos", "payload_rot", "action", "setpoint",
                                    "drone_point", "goal_index")}
        self.ticks = 0
        self.policy_every = max(1, int(round(args.physics_hz * MP.POLICY_DT)))

    # --- what each RaptorBackend asks for every physics step ---
    def target(self, i, t):
        if self.rl:
            sp = self.integrators[i]
            # RAPTOR flies the body origin; the setpoint is for the policy's point
            velocity = np.zeros(3) if "no_vel_ff" in DIAG else sp.velocity
            return sp.position - self.point_local, velocity, self.yaws[i]
        zero = np.zeros(3)
        if t < self.t_takeoff[0]:
            return self.ground[i], zero, self.yaws[i]
        if t < self.t_lift[0]:
            T = self.t_takeoff[1] - self.t_takeoff[0]
            p, v = min_jerk((t - self.t_takeoff[0]) / T)
            d = self.slack[i] - self.ground[i]
            return self.ground[i] + d * p, d * v / T, self.yaws[i]
        T = self.t_lift[1] - self.t_lift[0]
        p, v = min_jerk((t - self.t_lift[0]) / T)
        d = self.lifted[i] - self.slack[i]
        return self.slack[i] + d * p, d * v / T, self.yaws[i]

    # --- every physics step ---
    def on_physics_step(self, dt):
        t = self.backends[0].t
        phase = ("ground" if t < self.t_takeoff[0] else "take-off" if t < self.t_takeoff[1] else
                 "in position, cables slack" if t < self.t_lift[0] else "tension and lift" if t < self.t_lift[1] else
                 "hold" if t < self.t_rl else self.phase)
        if phase != self.phase and not self.rl:
            self.phase = phase
            print(f"[marl] t = {t:6.2f} s  {phase}")
        if t < self.t_rl or args.no_rl or self.handover_ok is False:
            return
        self.ticks += 1
        if self.rl and (self.ticks - 1) % self.policy_every != 0:
            return
        payload_pos, payload_rot, drones = self.read_state()
        if not self.rl:
            self.hand_over(t, payload_pos, payload_rot, drones)
            if not self.rl:
                return
        self.update_goal(t, payload_pos)
        actions = []
        for i, (pos, rot, lin_vel, ang_vel) in enumerate(drones):
            obs = self.observations[i].push(
                payload_pos=payload_pos, payload_rot=payload_rot, drone_pos=pos, drone_rot=rot, drone_lin_vel=lin_vel,
                drone_ang_vel=ang_vel, goal_pos=self.goal_pos, goal_rot=self.goal_rot)
            action = self.policies[i](obs)
            if "filter" in DIAG:
                a = float(DIAG["filter"])
                self.filtered[i] = action if self.filtered[i] is None else a * action + (1 - a) * self.filtered[i]
                action = self.filtered[i]
            self.integrators[i].step(action, pos)
            actions.append(action)
        L = self.log
        L["t"].append(t)
        L["goal_pos"].append(self.goal_pos.copy())
        L["goal_rot"].append(self.goal_rot.copy())
        L["payload_pos"].append(payload_pos)
        L["payload_rot"].append(payload_rot)
        L["action"].append(np.array(actions))
        L["setpoint"].append(np.array([sp.position for sp in self.integrators]))
        L["drone_point"].append(np.array([d[0] for d in drones]))
        L["goal_index"].append(self.goal_index)

    def read_state(self):
        """Payload pose and, per drone, what the policy observes: all in the world frame."""
        pp, pq = self.rig_rt._pose(self.rig_rt.payload_h)
        payload_rot = Rotation.from_quat(pq).as_matrix()
        drones = []
        for b in self.backends:
            s = b.vehicle.state
            rot = Rotation.from_quat(s.attitude).as_matrix()  # body FLU -> world ENU
            ang_vel = rot @ np.asarray(s.angular_velocity)  # Pegasus reports body rates; the policy wants world
            arm = rot @ self.point_local
            drones.append((np.asarray(s.position) + arm, rot, np.asarray(s.linear_velocity) + np.cross(ang_vel, arm),
                           ang_vel))
        return pp, payload_rot, drones

    def hand_over(self, t, payload_pos, payload_rot, drones):
        """Switch from the scripted targets to the policy, if the rig is in the state its episodes start from."""
        anchors, mounts = self.rig_rt.endpoints()
        spans = np.linalg.norm(mounts - anchors, axis=1)
        taut = spans >= self.rig_rt.L - TAUT_TOLERANCE
        bottom = payload_pos[2] - rig.payload.height / 2
        self.handover_ok = bool(taut.all() and bottom > 0.2)
        print(f"[marl] t = {t:6.2f} s  hand-over check: cable spans {np.round(spans, 3)} m of {self.rig_rt.L} m "
              f"({'all taut' if taut.all() else 'NOT all taut'}), payload {bottom:.2f} m above the ground")
        if not self.handover_ok:
            print("[marl] hand-over refused: RAPTOR keeps holding the formation")
            return
        for i, (pos, _, _, _) in enumerate(drones):
            self.integrators[i].seed(pos)
            self.observations[i].reset()
        yaw = math.atan2(payload_rot[1, 0], payload_rot[0, 0])
        self.goal_home = (payload_pos.copy(), MP.euler_xyz_to_matrix(0.0, 0.0, yaw))
        self.goal_pos, self.goal_rot = self.goal_home
        self.rl = True
        self.phase = "RL: holding the payload where it is"
        print(f"[marl] t = {t:6.2f} s  {self.phase}")

    def update_goal(self, t, payload_pos):
        if self.teleop is not None and (manual_goal or t >= self.t_goals + len(goals) * args.goal_time):
            # the keyboard moves it in 0.25 m steps, inside the box the policy's goals were drawn from in training
            wanted = self.goal_home[0] + self.teleop.goal
            self.goal_pos = np.clip(wanted, GOAL_BOX[0], GOAL_BOX[1])
            self.teleop.goal += self.goal_pos - wanted  # so a key at the edge does not wind the offset up
            return
        index = int((t - self.t_goals) // args.goal_time) if t >= self.t_goals else -1
        index = min(index, len(goals) - 1)
        if index != self.goal_index and index >= 0:
            g = goals[index]
            self.goal_pos = np.array(g[:3])
            self.goal_rot = MP.euler_xyz_to_matrix(*np.radians(g[3:]))
            if self.teleop is not None:  # so the keyboard later continues from this goal
                self.goal_home = (self.goal_pos.copy(), self.goal_rot)
                self.teleop.goal[:] = 0.0
            print(f"[marl] t = {t:6.2f} s  goal {index}: position {g[:3]} m, roll/pitch/yaw {g[3:]} deg "
                  f"({np.linalg.norm(self.goal_pos - payload_pos):.2f} m away)")
        self.goal_index = index


def main():
    n = rig.num_drones
    raptor = RaptorPolicy(args.raptor)

    # The formation with the cables just taut and the payload still on the ground: where the take-off is headed.
    rig.spawn.payload_clearance = 0.0
    vinfo = U.vehicle_info(rig, vehicle_cfg, os.path.join(C.GENERATED_DIR, "runs"), ROBOTS["Iris"], unique=True)
    taut_layout, warnings = U.rig_layout_for(rig, vinfo)
    for w in warnings:
        print(f"[marl] WARNING {w}")
    taut = [np.asarray(p, dtype=float) for p in taut_layout.drone_pos]
    yaws = list(taut_layout.drone_yaw)
    ground = [np.array([p[0], p[1], vinfo.spawn_height_on_ground]) for p in taut]
    slack = [p - np.array([0.0, 0.0, SLACK_MARGIN]) for p in taut]
    lift = args.lift_height - float(taut_layout.payload_pos[2])
    lifted = [p + np.array([0.0, 0.0, lift]) for p in taut]
    mount_local = np.asarray(vinfo.mount_local, dtype=float)
    # the point the policy was trained to call the drone's position, see marl_policy.TRAINED_ON
    point_local = MP.policy_point(args.trained_on, mount_local, vinfo.summary["com"])
    if "point" in DIAG:
        point_local = mount_local + np.array([0.0, 0.0, float(DIAG["point"])])
    # the rig is authored with the drones standing on the ground: the cables start slack
    spawn_layout = dataclasses.replace(
        taut_layout, drone_pos=ground,
        mounts_world=[g + U.G.rot_z(y) @ mount_local for g, y in zip(ground, yaws)])
    print(f"[marl] {n} x S500 on the ground {taut_layout.horizontal_distance:.3f} m from the payload axis, cables "
          f"{rig.cable.model} {rig.cable.length} m; taut at {taut[0][2]:.2f} m, lifted formation at {lifted[0][2]:.2f} m")

    timeline = omni.timeline.get_timeline_interface()
    pg = PegasusInterface()
    pg._world = World(physics_dt=1.0 / args.physics_hz, rendering_dt=1.0 / args.render_hz, stage_units_in_meters=1.0)
    world = pg.world
    GroundPlane(prim_path="/World/GroundPlane", size=100.0, color=np.array([0.4, 0.4, 0.45]))
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, "/World/Sun").CreateIntensityAttr(2500.0)
    UsdLux.DomeLight.Define(stage, "/World/Sky").CreateIntensityAttr(600.0)

    seq = Sequence(ground, slack, lifted, yaws, point_local, None, n)
    backends, drone_paths = [], []
    for i in range(n):
        backend_cls = Raptor100Hz if "raptor_100hz" in DIAG else RaptorBackend
        backend = backend_cls(raptor, (lambda k: (lambda t: seq.target(k, t)))(i), np.zeros(3), args.physics_hz)
        cfg = MultirotorConfig()
        cfg.backends = [backend]
        cfg.sensors = []
        tau = vinfo.motor_time_constant
        cfg.thrust_curve = (LaggedThrustCurve(vinfo.thrust_curve, tau) if tau > 0
                            else QuadraticThrustCurve(vinfo.thrust_curve))
        cfg.drag = LinearDrag(vinfo.linear_drag)
        path = f"/World/drone{i}"
        Multirotor(path, vinfo.usd_path, i, [float(x) for x in ground[i]],
                   [0.0, 0.0, math.sin(yaws[i] / 2), math.cos(yaws[i] / 2)], config=cfg)
        backends.append(backend)
        drone_paths.append(path)
    handles = U.author_rig(stage, rig, vinfo, spawn_layout, drone_paths)
    rig_rt = RigRuntime(stage, handles, rig, 1.0 / args.physics_hz, time_fn=lambda: pg.world.current_time)
    seq.rig_rt, seq.backends = rig_rt, backends
    if not args.no_rl:
        seq.policies = [MP.OnnxPolicy(os.path.join(args.models, f"policy_falcon{i + 1}.onnx")) for i in range(n)]
    marker = GoalMarker(stage, rig.payload.radius, rig.payload.height, n)
    teleop = None
    if not args.headless:
        teleop = KeyboardTeleop(log=lambda m: print(m.replace("formation goal offset", "payload goal moved by")),
                                z_min=-(args.lift_height - 0.3))
        seq.teleop = teleop

    world.reset()
    rig_rt.initialize()
    world.add_physics_callback("castor_rig", rig_rt.on_physics_step)
    world.add_physics_callback("marl_sequence", seq.on_physics_step)

    centre = np.array([0.0, 0.0, 0.5 * (args.lift_height + lifted[0][2])])
    span = taut_layout.horizontal_distance + 1.0
    eye = centre + np.array([1.2 + 1.5 * span, -(1.2 + 1.5 * span), 0.6 + 0.8 * span])
    if not args.headless:
        from isaacsim.core.utils.viewports import set_camera_view

        set_camera_view(eye=eye, target=centre, camera_prim_path="/OmniverseKit_Persp")

    def on_frame():
        rig_rt.update_visuals()
        if seq.rl:
            marker.show(seq.goal_pos, seq.goal_rot, [sp.position for sp in seq.integrators])

    if not args.headless:
        on_frame()
        teleop.start()
    duration = 3600.0 if manual_goal else seq.duration
    timeline.play()
    wall0 = time.perf_counter()
    steps = run_physics(world, simulation_app, duration, args.physics_hz, render=not args.headless,
                        render_hz=args.render_hz, realtime=not args.fast, on_frame=on_frame)
    wall = time.perf_counter() - wall0
    sim_time = world.current_time

    out = {}
    for i, b in enumerate(backends):
        for k, v in b.log.items():
            out[f"v{i}_{k}"] = np.array(v)
    metrics = rig_rt.metrics()
    out.update(rig_rt.arrays(metrics))
    out.update({f"marl_{k}": np.array(v) for k, v in seq.log.items()})
    meta = {"args": vars(args), "rig": C.to_dict(rig), "vehicle": C.to_dict(vehicle_cfg), "goals": goals,
            "phases": {"takeoff": seq.t_takeoff, "lift": seq.t_lift, "rl": seq.t_rl, "goals": seq.t_goals},
            "sim_time": sim_time, "steps": steps, "wall_time": wall}
    os.makedirs(os.path.dirname(args.log), exist_ok=True)
    np.savez(args.log, meta=json.dumps(meta, default=str), **out)
    print(f"[marl] log written to {args.log}")
    ok = report(seq, metrics, backends, sim_time, wall)

    if args.snapshot:
        save_snapshot(args.snapshot, on_frame, eye, centre)
    if args.keep_open and not args.headless:
        print("[marl] goals done; the keyboard now moves the goal. Close the window to exit.")
        run_physics(world, simulation_app, 3600.0 + sim_time, args.physics_hz, render=True, render_hz=args.render_hz,
                    realtime=not args.fast, on_frame=on_frame)
    timeline.stop()
    if not ok:
        os._exit(1)  # simulation_app.close() ends the process itself, with status 0
    simulation_app.close()


def save_snapshot(path, on_frame, eye, centre):
    """Headless: render the final scene once through a replicator camera."""
    import omni.replicator.core as rep

    on_frame()
    camera = rep.create.camera(position=tuple(float(x) for x in eye), look_at=tuple(float(x) for x in centre))
    product = rep.create.render_product(camera, (1280, 800))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach([product])
    for _ in range(20):
        rep.orchestrator.step(rt_subframes=4, pause_timeline=False)
    from PIL import Image

    Image.fromarray(np.asarray(rgb.get_data())[..., :3]).save(path)
    print(f"[marl] snapshot written to {path}")


def report(seq, M, backends, sim_time, wall):
    print(f"[marl] simulated {sim_time:.2f} s ({wall:.1f} s wall, {sim_time / max(wall, 1e-9):.2f}x real time)")
    t = M["t"]
    stretch = M["stretch"]

    def window(t0, t1):
        return (t >= t0) & (t < t1)

    lift_w = window(seq.t_lift[0], seq.t_rl)
    first_taut = t[lift_w][np.argmax((stretch[lift_w] > -TAUT_TOLERANCE).all(axis=1))] if lift_w.any() else float("nan")
    airborne = t[np.argmax(~M["payload_contact"] & (t > seq.t_lift[0]))]
    print(f"[marl] all cables taut at t = {first_taut:.2f} s, payload off the ground at t = {airborne:.2f} s; "
          f"payload height at hand-over {M['payload_pos'][np.searchsorted(t, seq.t_rl) - 1, 2]:.3f} m")
    if args.no_rl:
        return True
    if not seq.rl:
        print("[marl] FAIL: the policy never took over")
        return False

    ok = True
    L = {k: np.array(v) for k, v in seq.log.items()}
    rl_w = t >= L["t"][0]
    slack = (stretch[rl_w] < -TAUT_TOLERANCE).any(axis=1).mean()
    drones = M["drone_pos"][rl_w]
    pairs = [np.linalg.norm(drones[:, i] - drones[:, j], axis=1).min() for i in range(seq.n) for j in range(i)]
    tilt = speed = 0.0
    for b in backends:
        under = np.array(b.log["t"]) >= L["t"][0]
        q = np.array(b.log["quat"])[under]  # x y z w
        tilt = max(tilt, float(np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 0] ** 2 + q[:, 1] ** 2), -1, 1))).max()))
        speed = max(speed, float(np.linalg.norm(np.array(b.log["vel"])[under], axis=1).max()))
    contact = bool(M["payload_contact"][rl_w].any())
    print(f"[marl] under the policy: a cable slack {100 * slack:.1f}% of the time, closest two drones {min(pairs):.2f} m, "
          f"largest drone tilt {tilt:.1f} deg, fastest drone {speed:.2f} m/s, payload touched the ground: {contact}, "
          f"largest |action| {np.abs(L['action']).max():.2f}")
    ok &= not contact and min(pairs) > 0.6

    def errors(mask):
        pos = np.linalg.norm(L["goal_pos"][mask] - L["payload_pos"][mask], axis=1)
        rel = np.einsum("kij,klj->kil", L["goal_rot"][mask], L["payload_rot"][mask])
        ang = np.degrees(np.arccos(np.clip((np.trace(rel, axis1=1, axis2=2) - 1) / 2, -1, 1)))
        return pos, ang

    for index in sorted(set(L["goal_index"].tolist())):
        mask = L["goal_index"] == index
        pos, ang = errors(mask)
        tail = max(1, int(2.0 / MP.POLICY_DT))  # the last two seconds on this goal
        name = "holding at hand-over" if index < 0 else f"goal {index} {goals[index][:3]}"
        passed = pos[-tail:].mean() < 0.15
        ok &= passed
        print(f"[marl] {name}: start {pos[0]:.2f} m / {ang[0]:.0f} deg away, last 2 s mean {pos[-tail:].mean():.3f} m / "
              f"{ang[-tail:].mean():.1f} deg  {'ok' if passed else 'MISSED'}")
    print("[marl] PASS" if ok else "[marl] FAIL")
    return ok


if __name__ == "__main__":
    try:
        main()
    except Exception:  # Kit catches uncaught exceptions and still exits 0
        import traceback

        traceback.print_exc()
        os._exit(1)
