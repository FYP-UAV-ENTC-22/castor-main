"""Fly N RAPTOR-controlled drones carrying one cable-suspended payload in Isaac Sim (Pegasus, no PX4), flycrane-style.

The rig comes from components/simulation/assets/config/payload_rig.yaml (payload, cables, formation, mount height,
release mechanisms) and s500.yaml (airframe); override any key with --set / --vset. Every drone flies its own,
unchanged RaptorBackend (raptor_backend.py). RAPTOR knows nothing about the payload: each drone just holds its own
point of the formation, and the trajectory moves the whole formation.

    source env/env.sh
    cd components/simulation/tests/raptor
    python raptor_payload.py --headless --trajectory hover
    python raptor_payload.py --trajectory waypoints --keep_open            # with the window
    python raptor_payload.py --headless --set cable.model=rope --release 12:0:drone
    python raptor_payload.py --headless --no_payload --set num_drones=1    # the airframe alone

Logs go to runs/payload_<trajectory>_<cable>_n<N>.npz: per-drone RAPTOR logs (v<i>_*), rig state (rig_*: payload
pose and velocity, cable end points, distance, stretch, tension), and a JSON "meta" entry with the resolved configs.
"""

import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CASTOR_ROOT = os.environ.get("CASTOR_ROOT") or os.path.abspath(os.path.join(HERE, "../../../.."))
PEGASUS_EXT = os.path.join(CASTOR_ROOT, "components/simulation/pegasus_simulator/extensions/pegasus.simulator")
ASSETS = os.path.join(CASTOR_ROOT, "components/simulation/assets")
DEFAULT_POLICY = os.path.join(CASTOR_ROOT, "components/vehicle/PX4-Autopilot/src/modules/mc_raptor/blob/policy.tar")
sys.path.insert(0, ASSETS)

from castor_assets import config as C  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--trajectory", choices=["hover", "waypoints", "manual"], default="hover",
                    help="hover = hold the spawn formation; waypoints = slow min-jerk moves of the whole formation; "
                         "manual = fly the formation with the keyboard (GUI only, see keyboard_teleop.py)")
parser.add_argument("--duration", type=float, default=None,
                    help="s; default: 15 (hover), the waypoint plan (waypoints), until the window closes (manual)")
parser.add_argument("--physics_hz", type=float, default=400.0)
parser.add_argument("--render_hz", type=float, default=50.0, help="GUI redraw rate (physics steps per frame = ratio)")
parser.add_argument("--fast", action="store_true", help="GUI: run as fast as possible instead of real time")
parser.add_argument("--keep_open", action="store_true", help="GUI: pause at the end and keep the window open")
parser.add_argument("--follow", action="store_true", help="GUI: camera follows the payload")
parser.add_argument("--rig", default="payload_rig.yaml", help="rig config (path, or a name in assets/config/)")
parser.add_argument("--vehicle", default=None, help="vehicle config; default: the rig's vehicle_config")
parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="rig override, repeatable")
parser.add_argument("--vset", action="append", default=[], metavar="KEY=VALUE", help="vehicle override, repeatable")
parser.add_argument("--release", action="append", default=[], metavar="T:CABLE:END",
                    help="command a release at time T (s) on cable index CABLE, END = drone|payload; repeatable")
parser.add_argument("--no_payload", action="store_true", help="drones only, same layout, no payload or cables")
parser.add_argument("--policy", default=DEFAULT_POLICY)
parser.add_argument("--log", default=None)
parser.add_argument("--profile", action="store_true", help="print a cProfile summary of the stepping loop")
args = parser.parse_args()

# config mistakes fail here, before Isaac Sim starts
rig = C.load_rig(args.rig, args.set)
vehicle_cfg = C.load_vehicle(args.vehicle or rig.vehicle_config, args.vset)
releases = []
for item in args.release:
    t_s, cable_s, end = item.split(":")
    releases.append((float(t_s), int(cable_s), end))
if args.trajectory == "manual" and args.headless:
    parser.error("--trajectory manual needs the window")
if args.log is None:
    tag = "nopayload" if args.no_payload else rig.cable.model
    args.log = os.path.join(HERE, "runs", f"payload_{args.trajectory}_{tag}_n{rig.num_drones}.npz")

sys.stdout.reconfigure(line_buffering=True)  # SimulationApp.close() hard-exits and would drop buffered output

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("omni.isaac.dynamic_control")  # Pegasus 5.1 still uses it
simulation_app.update()

sys.path.insert(0, PEGASUS_EXT)
sys.path.insert(0, os.path.join(HERE, ".deps"))  # pymavlink, see README
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import torch  # noqa: E402
from isaacsim.core.api.objects import GroundPlane  # noqa: E402
from isaacsim.core.api.world import World  # noqa: E402
from pxr import UsdGeom, UsdLux  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from pegasus.simulator.logic.dynamics import LinearDrag  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.params import ROBOTS  # noqa: E402

from castor_assets import usd_build as U  # noqa: E402
from castor_assets.runtime import RigRuntime  # noqa: E402
from keyboard_teleop import KeyboardTeleop  # noqa: E402
from raptor_backend import RaptorBackend  # noqa: E402
from raptor_policy import RaptorPolicy  # noqa: E402
from sim_loop import run_physics  # noqa: E402

torch.set_num_threads(1)


class LaggedThrustCurve(QuadraticThrustCurve):
    """QuadraticThrustCurve with a first-order motor lag (propulsion.motor_time_constant)."""

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


class FormationTarget:
    """The offset every drone's RAPTOR target follows; swapping .fn retargets all drones at once."""

    def __init__(self, fn):
        self.fn = fn

    def __call__(self, t):
        return self.fn(t)


def formation_plan(kind):
    """Returns f(t) -> (offset ENU, velocity ENU) applied to every drone's home point, the duration, and the plan."""
    if kind == "hover":
        return (lambda t: (np.zeros(3), np.zeros(3))), 15.0, []
    if kind == "manual":
        return None, 3600.0, []
    hold, move = 4.0, 6.0
    points = [np.zeros(3), np.array([1.0, 0.0, 0.0]), np.array([1.0, 1.0, 0.3]), np.zeros(3)]
    segments, t = [], hold
    for a, b in zip(points[:-1], points[1:]):
        segments.append((t, t + move, a, b))
        t += move + hold

    def f(t):
        for t0, t1, a, b in segments:
            if t < t0:
                return a.copy(), np.zeros(3)
            if t <= t1:
                s = (t - t0) / (t1 - t0)
                pos = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
                vel = (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / (t1 - t0)
                return a + (b - a) * pos, (b - a) * vel
        return segments[-1][3].copy(), np.zeros(3)

    return f, t, segments


def main():
    plan, plan_duration, segments = formation_plan(args.trajectory)
    duration = args.duration or plan_duration
    teleop = KeyboardTeleop(z_min=-(rig.spawn.payload_clearance + 0.1)) if not args.headless else None
    offset = FormationTarget(plan if plan is not None else teleop.offset)
    policy = RaptorPolicy(args.policy)

    vinfo = U.vehicle_info(rig, vehicle_cfg, os.path.join(C.GENERATED_DIR, "runs"), ROBOTS["Iris"], unique=True)
    layout, warnings = U.rig_layout_for(rig, vinfo)
    for w in warnings:
        print(f"[payload] WARNING {w}")
    if vinfo.kind == "s500":
        s = vinfo.summary
        print(f"[payload] S500: {s['total_mass']:.3f} kg, inertia {np.round(s['inertia_diag'], 5)}, "
              f"T/W {s['thrust_to_weight']:.2f}, hover throttle {s['hover_throttle']:.3f}, USD {vinfo.usd_path}")
    print(f"[payload] {rig.num_drones} x {rig.vehicle}, cable {rig.cable.model} {rig.cable.length} m, "
          f"drones {layout.horizontal_distance:.3f} m out, cable angle {math.degrees(layout.cable_angle):.1f} deg, "
          f"static tension {layout.static_tension:.2f} N/cable, mount {vinfo.mount_local.round(4)} (body)")

    timeline = omni.timeline.get_timeline_interface()
    pg = PegasusInterface()
    pg._world = World(physics_dt=1.0 / args.physics_hz, rendering_dt=1.0 / args.render_hz, stage_units_in_meters=1.0)
    world = pg.world
    GroundPlane(prim_path="/World/GroundPlane", size=100.0, color=np.array([0.4, 0.4, 0.45]))
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, "/World/Sun").CreateIntensityAttr(2500.0)
    UsdLux.DomeLight.Define(stage, "/World/Sky").CreateIntensityAttr(600.0)

    backends, drone_paths = [], []
    for i in range(rig.num_drones):
        yaw = layout.drone_yaw[i]
        home = layout.drone_pos[i]
        traj = (lambda yaw_i: (lambda t: (*offset(t), yaw_i)))(yaw)
        backend = RaptorBackend(policy, traj, home, args.physics_hz)
        cfg = MultirotorConfig()
        cfg.backends = [backend]
        cfg.sensors = []
        if vinfo.thrust_curve:
            tau = vinfo.motor_time_constant
            cfg.thrust_curve = LaggedThrustCurve(vinfo.thrust_curve, tau) if tau > 0 else QuadraticThrustCurve(
                vinfo.thrust_curve)
            cfg.drag = LinearDrag(vinfo.linear_drag)
        path = f"/World/drone{i}"
        Multirotor(path, vinfo.usd_path, i, [float(x) for x in home], [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)],
                   config=cfg)
        backends.append(backend)
        drone_paths.append(path)

    rig_rt = None
    if not args.no_payload:
        handles = U.author_rig(stage, rig, vinfo, layout, drone_paths)
        rig_rt = RigRuntime(stage, handles, rig, 1.0 / args.physics_hz, time_fn=lambda: pg.world.current_time)
    if teleop is not None:
        teleop.rig_rt = rig_rt

    world.reset()
    if rig_rt is not None:
        rig_rt.initialize()
        world.add_physics_callback("castor_rig", rig_rt.on_physics_step)
        for t_cmd, cable, end in releases:
            rig_rt.pending.append([t_cmd + rig.release.actuation_delay, cable, end, t_cmd])
            if not rig_rt.h.release_ends[("drone", "payload").index(end)]:
                raise ValueError(f"--release {t_cmd}:{cable}:{end}: release.location is {rig.release.location!r}")

    # mc_raptor assumes PX4 quad X (1 FR, 2 BL, 3 FL, 4 BR)
    cache = UsdGeom.XformCache()
    body_inv = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(f"{drone_paths[0]}/body")).GetInverse()
    expected = [(1, -1), (-1, 1), (1, 1), (-1, -1)]
    for r in range(4):
        xyz = (cache.GetLocalToWorldTransform(stage.GetPrimAtPath(f"{drone_paths[0]}/rotor{r}")) * body_inv).ExtractTranslation()
        ok = (np.sign(xyz[0]), np.sign(xyz[1])) == expected[r]
        print(f"[payload] rotor{r} body xyz = ({xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f})  quad-X match: {ok}")

    # camera: the payload-to-drone column, plus the middle of the waypoint plan when there is one
    centre = layout.payload_pos + np.array([0.0, 0.0, 0.55])
    if segments:
        ends = np.array([b for _, _, _, b in segments])
        centre = centre + (ends.max(axis=0) + ends.min(axis=0)) / 2
    if not args.headless:
        from isaacsim.core.utils.viewports import set_camera_view

        span = max(layout.horizontal_distance, 0.5) + (0.7 if segments else 0.0)
        eye = centre + np.array([1.5 + 1.6 * span, -(1.5 + 1.6 * span), 0.8 + 0.9 * span])
        set_camera_view(eye=eye, target=centre, camera_prim_path="/OmniverseKit_Persp")

    def on_frame():
        if rig_rt is not None:
            rig_rt.update_visuals()
            if args.follow:
                p = rig_rt.payload_position()
                set_camera_view(eye=p + (eye - centre), target=p + np.array([0, 0, 0.6]),
                                camera_prim_path="/OmniverseKit_Persp")

    if rig_rt is not None and not args.headless:
        on_frame()
    if teleop is not None:
        teleop.start()
    timeline.play()
    wall0 = time.perf_counter()
    if args.profile:
        import cProfile
        import pstats

        prof = cProfile.Profile()
        prof.enable()
    steps = run_physics(world, simulation_app, duration, args.physics_hz, render=not args.headless,
                        render_hz=args.render_hz, realtime=not args.fast, on_frame=on_frame,
                        before_step=rig_rt.apply_due_releases if rig_rt is not None else None)
    wall = time.perf_counter() - wall0
    if args.profile:
        prof.disable()
        pstats.Stats(prof).sort_stats("tottime").print_stats(25)
    sim_time = world.current_time
    if args.keep_open and not args.headless:
        timeline.pause()
    else:
        timeline.stop()

    out = {}
    for i, b in enumerate(backends):
        for k, v in b.log.items():
            out[f"v{i}_{k}"] = np.array(v)
    metrics = rig_rt.metrics() if rig_rt is not None else None
    if rig_rt is not None:
        out.update(rig_rt.arrays(metrics))
    meta = {
        "args": vars(args), "rig": C.to_dict(rig), "vehicle": C.to_dict(vehicle_cfg), "vehicle_summary": vinfo.summary,
        "sim_time": sim_time, "steps": steps, "wall_time": wall, "duration": duration,
        "layout": {"drone_pos": [list(map(float, p)) for p in layout.drone_pos], "drone_yaw": layout.drone_yaw,
                   "payload_pos": list(map(float, layout.payload_pos)), "static_tension": layout.static_tension,
                   "cable_angle_deg": math.degrees(layout.cable_angle), "mount_local": list(map(float, vinfo.mount_local))},
        "release_events": rig_rt.events if rig_rt is not None else [],
        "segments": [(t0, t1, list(map(float, a)), list(map(float, b))) for t0, t1, a, b in segments],
    }
    os.makedirs(os.path.dirname(args.log), exist_ok=True)
    np.savez(args.log, meta=json.dumps(meta, default=str), **out)
    print(f"[payload] log written to {args.log}")
    report(backends, rig_rt, metrics, layout, sim_time, steps, wall, duration)

    if args.keep_open and not args.headless:
        # from here the keyboard flies the formation, starting where the scripted run ended
        t_end = backends[0].t
        teleop.pos = np.array(offset(t_end)[0], dtype=float)
        teleop.goal = teleop.pos.copy()  # separate arrays: a key press must move the goal, not the current target
        teleop.t = t_end
        offset.fn = teleop.offset
        print("[payload] paused at the end; press Play to keep flying (keyboard: see keyboard_teleop.py), "
              "close the window to exit")
        while simulation_app.is_running():
            if rig_rt is not None:
                rig_rt.apply_due_releases()
            on_frame()
            simulation_app.update()
    simulation_app.close()


def report(backends, rig_rt, L, layout, sim_time, steps, wall, duration):
    n_log = [len(b.log["t"]) for b in backends]
    settle = min(3.0, sim_time / 2)  # statistics skip the start-up transient
    print(f"[payload] simulated {sim_time:.4f} s of {duration:.4f} s in {steps} physics steps ({wall:.1f} s wall, "
          f"{sim_time / max(wall, 1e-9):.2f}x real time); log lengths drones {sorted(set(n_log))}"
          + (f", rig {len(rig_rt.log['t'])}" if rig_rt is not None else ""))
    for i, b in enumerate(backends):
        t = np.array(b.log["t"])
        pos, tgt = np.array(b.log["pos"]), np.array(b.log["target"])
        err = np.linalg.norm(pos - tgt, axis=1)
        tilt = np.degrees(np.arccos(np.clip([Rotation.from_quat(q).as_matrix()[2, 2] for q in b.log["quat"]], -1, 1)))
        m = t > settle
        print(f"[payload] drone {i}: pos err RMS after {settle:.1f} s {np.sqrt(np.mean(err[m] ** 2)):.3f} m, max {err[m].max():.3f} m, "
              f"final {err[-1]:.3f} m; max tilt {tilt.max():.1f} deg; throttle mean {np.mean(b.log['action']):.3f} "
              f"max {np.max(b.log['action']):.3f}")
    if rig_rt is None:
        return
    t = L["t"]
    # the formation offset the drones were flying, from drone 0's logged target (works for every trajectory mode)
    b0 = backends[0]
    tb = np.array(b0.log["t"]) + 1.0 / args.physics_hz
    off = np.array(b0.log["target"]) - np.asarray(layout.drone_pos[0])
    ref = layout.payload_pos + np.stack([np.interp(t, tb, off[:, k]) for k in range(3)], axis=1)
    perr = np.linalg.norm(L["payload_pos"] - ref, axis=1)
    tilt = np.degrees(np.arccos(np.clip([Rotation.from_quat(q).as_matrix()[2, 2] for q in L["payload_quat"]], -1, 1)))
    m = t > settle
    print(f"[payload] payload: offset from its rigid-formation point after {settle:.1f} s mean {perr[m].mean():.3f} m, "
          f"max {perr[m].max():.3f} m (sag {np.mean(ref[m, 2] - L['payload_pos'][m, 2]) * 1e3:.1f} mm); "
          f"max tilt {tilt.max():.1f} deg; min z {L['payload_pos'][:, 2].min():.3f} m; "
          f"ground contact steps {int(L['payload_contact'].sum())}")
    att = L["attached"].all(axis=2)
    for i in range(rig_rt.n):
        a = att[:, i] & m
        if not a.any():
            continue
        st = L["stretch"][a, i] * 1e3
        tn = L["tension_newton"][a, i]
        line = (f"[payload] cable {i}: end-to-end minus length mean {st.mean():+.2f} mm, max {st.max():+.2f} mm, "
                f"slack (> 1 mm short) {100 * np.mean(st < -1.0):.1f}% of time; tension (Newton) mean "
                f"{np.nanmean(tn):.2f} N, min {np.nanmin(tn):.2f}, max {np.nanmax(tn):.2f}")
        if np.isfinite(L["tension_spring"][a, i]).any():
            line += f"; spring formula mean {np.nanmean(L['tension_spring'][a, i]):.2f} N"
        if "gap_drone_end" in L:
            line += (f"; end-joint gap max payload {1e3 * L['gap_payload_end'][a, i].max():.3f} mm, "
                     f"drone {1e3 * L['gap_drone_end'][a, i].max():.3f} mm")
        print(line)
    print(f"[payload] static tension estimate {layout.static_tension:.2f} N per cable")
    for ev in rig_rt.events:
        print(f"[payload] release: commanded t={ev[0]:.3f} s, applied t={ev[1]:.4f} s, cable {ev[2]} {ev[3]} end")


if __name__ == "__main__":
    # Kit swallows uncaught exceptions and exits 0, so fail loudly ourselves
    try:
        main()
    except Exception:
        import traceback

        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
