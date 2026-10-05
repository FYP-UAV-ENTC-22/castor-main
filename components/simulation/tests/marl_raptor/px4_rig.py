"""The simulator side of the PX4 run: three S500s flown by PX4 SITL (px4_sitl_raptor), cables and the payload.

Isaac Sim does the physics and the sensors here and nothing else. Each drone has its own PX4 instance, started by
this script, which gets IMU, barometer, magnetometer and GPS from Pegasus, runs its own estimator, and drives the
rotors, in a PX4 mode (take-off, hold) or in RAPTOR (mc_raptor). Everything that commands or observes a drone goes
through PX4's uXRCE-DDS client, one UDP port and one namespace per instance:

    drone i (0-based):  simulator link tcp 4560 + i,  uXRCE-DDS agent udp 8888 + i,  topics /px4_<i + 1>/fmu/...

The payload carries no flight controller. Its pose is taken from the simulation, given RTK-grade noise, and sent as
JSON over UDP to 127.0.0.1:--payload_port at 50 Hz for payload_relay.py to publish as /team/payload/odom. The goal
the policy is flying to arrives the same way on --goal_port and is drawn as a green disc.

The drones start on the ground around the payload with slack cables, disarmed. See README.md for the full sequence.

    source env/env.sh
    cd components/simulation/tests/marl_raptor
    python px4_rig.py                     # with the window
    python px4_rig.py --headless --duration 120
"""

import argparse
import dataclasses
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CASTOR_ROOT = os.environ.get("CASTOR_ROOT") or os.path.abspath(os.path.join(HERE, "../../../.."))
PEGASUS_EXT = os.path.join(CASTOR_ROOT, "components/simulation/pegasus_simulator/extensions/pegasus.simulator")
ASSETS = os.path.join(CASTOR_ROOT, "components/simulation/assets")
RAPTOR_TESTS = os.path.join(HERE, "../raptor")
sys.path.insert(0, ASSETS)

from castor_assets import config as C  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--duration", type=float, default=None, help="simulated seconds; default: until the window closes")
parser.add_argument("--px4", default=os.path.expanduser("~/Documents/Repos/PX4-Autopilot"),
                    help="PX4-Autopilot checkout with build/px4_sitl_raptor")
parser.add_argument("--takeoff_alt", type=float, default=1.3,
                    help="PX4 take-off height above the ground; the cables go taut at about 1.9 m")
parser.add_argument("--ros_domain", type=int, default=20, help="DDS domain PX4 publishes into (the CASTOR stack's)")
parser.add_argument("--physics_hz", type=float, default=250.0,
                    help="physics, sensor and PX4 gyro rate (IMU_GYRO_RATEMAX). RAPTOR was trained at 100 Hz and runs "
                         "its network at this rate divided by a whole number")
parser.add_argument("--payload_noise", type=float, default=1.0,
                    help="scale on the noise added to the payload pose sent out (1 = 1 cm and 0.5 deg per sample)")
parser.add_argument("--payload_port", type=int, default=14600)
parser.add_argument("--goal_port", type=int, default=14601)
parser.add_argument("--trained_on", choices=("falcon", "s500"), default="falcon",
                    help="the task the policy to be flown was trained on: picks the rig. Give px4_flight.py the same")
parser.add_argument("--rig", default=None,
                    help="rig config (path, or a name in assets/config/); default: the one --trained_on was trained on")
parser.add_argument("--vehicle", default=None, help="vehicle config; default: the rig's vehicle_config")
parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="rig override, repeatable")
parser.add_argument("--vset", action="append", default=[], metavar="KEY=VALUE", help="vehicle override, repeatable")
parser.add_argument("--render_hz", type=float, default=50.0)
parser.add_argument("--fast", action="store_true", help="GUI: run as fast as possible instead of real time")
args = parser.parse_args()

sys.path.insert(0, HERE)
import marl_policy as MP  # noqa: E402  -- NumPy only

rig = C.load_rig(args.rig or MP.TRAINED_ON[args.trained_on]["rig"], args.set)
vehicle_cfg = C.load_vehicle(args.vehicle or rig.vehicle_config, args.vset)
if rig.vehicle != "s500":
    parser.error("this run needs the S500 (its landing gear and cable mount)")
PX4_BIN = os.path.join(args.px4, "build/px4_sitl_raptor/bin/px4")
RAPTOR_CHECKPOINT = os.path.join(args.px4, "src/modules/mc_raptor/blob/policy.tar")
for needed in (PX4_BIN, RAPTOR_CHECKPOINT):
    if not os.path.exists(needed):
        parser.error(f"{needed} not found: build px4_sitl_raptor in {args.px4} (make px4_sitl_raptor)")

sys.stdout.reconfigure(line_buffering=True)  # SimulationApp.close() hard-exits and would drop buffered output

# PX4's IMU_GYRO_RATEMAX is set to this. mc_raptor leaves its mode if the gyro sample is older than 10 ms.
PHYSICS_HZ = args.physics_hz
PAYLOAD_HZ = 50.0
# RTK-grade GNSS: Pegasus' defaults model a metre-level receiver.
RTK_GPS = {"eph": 0.02, "epv": 0.03, "fix_type": 6, "gps_xy_random_walk": 0.0, "gps_z_random_walk": 0.0,
           "gps_xy_noise_density": 2.0e-5, "gps_z_noise_density": 4.0e-5, "gps_vxy_noise_density": 0.02,
           "gps_vz_noise_density": 0.04, "sattelites_visible": 18}
PAYLOAD_POS_NOISE = 0.01  # m, per axis, on the pose sent out
PAYLOAD_ROT_NOISE = math.radians(0.5)


def px4_parameters(vinfo):
    """What differs from PX4's none_iris defaults: RAPTOR, no RC or ground station, the S500's rotor layout."""
    arm = abs(vinfo.geometry.rotor_xy[0][0])
    params = {
        "MC_RAPTOR_ENABLE": 1,
        "MC_RAPTOR_OFFB": 0,  # RAPTOR is its own mode and holds position until setpoints arrive
        "MC_RAPTOR_INTREF": 0,
        "IMU_GYRO_RATEMAX": int(PHYSICS_HZ),
        "NAV_DLL_ACT": 0,  # no ground station link
        "NAV_RCL_ACT": 0,
        "COM_RC_IN_MODE": 4,  # no sticks
        "COM_DISARM_LAND": -1,  # the landing detector must not disarm a drone hanging on a cable
        "COM_DISARM_PRFLT": -1,
        "MIS_TAKEOFF_ALT": args.takeoff_alt,
        "MPC_TKO_SPEED": 0.7,
        "EKF2_HGT_REF": 1,  # GNSS height: with RTK it beats the barometer
        "EKF2_GPS_P_NOISE": 0.05,
        "EKF2_GPS_V_NOISE": 0.1,
        "UXRCE_DDS_SYNCT": 0,  # the agent's clock is the host's, PX4's is the simulation's
    }
    # PX4 quad-X: 0 front-right, 1 back-left, 2 front-left, 3 back-right; PX4's y is to the right
    for i, (px, py, km) in enumerate(((arm, arm, 0.05), (-arm, -arm, 0.05), (arm, -arm, -0.05), (-arm, arm, -0.05))):
        params.update({f"CA_ROTOR{i}_PX": px, f"CA_ROTOR{i}_PY": py, f"CA_ROTOR{i}_KM": km})
    hover = (vinfo.total_mass + rig.payload.mass / rig.num_drones) * 9.81 / vinfo.max_thrust_total
    # Pegasus maps a motor command u to 1000 u + 100 rad/s and thrust goes with speed squared
    params["MPC_THR_HOVER"] = round((1100.0 * math.sqrt(hover) - 100.0) / 1000.0, 3)
    return params


class Px4Instances:
    """One px4_sitl_raptor process per drone, each in its own working directory."""

    def __init__(self, n, params):
        self.processes = []
        self.root = os.path.join(HERE, "px4_rootfs")
        romfs = os.path.join(args.px4, "ROMFS/px4fmu_common")
        for i in range(n):
            rootfs = os.path.join(self.root, str(i))
            shutil.rmtree(rootfs, ignore_errors=True)  # stale parameters would override the ones set here
            os.makedirs(os.path.join(rootfs, "raptor"))
            shutil.copy(RAPTOR_CHECKPOINT, os.path.join(rootfs, "raptor/policy.tar"))
            env = {
                "PATH": "/usr/local/bin:/usr/bin:/bin", "HOME": os.environ["HOME"],
                "PX4_SIM_MODEL": "none_iris",
                "ROS_DOMAIN_ID": str(args.ros_domain),
                "PX4_UXRCE_DDS_NS": f"px4_{i + 1}",
                "PX4_UXRCE_DDS_PORT": str(8888 + i),
            }
            env.update({f"PX4_PARAM_{k}": str(v) for k, v in params.items()})
            log = open(os.path.join(rootfs, "console.log"), "w")
            self.processes.append(subprocess.Popen(
                [PX4_BIN, romfs, "-s", os.path.join(romfs, "init.d-posix/rcS"), "-i", str(i), "-d"],
                cwd=rootfs, env=env, stdout=log, stderr=subprocess.STDOUT))
        print(f"[px4] started {n} PX4 instances; consoles in {self.root}/<i>/console.log")

    def stop(self):
        for p in self.processes:
            p.terminate()
        for p in self.processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()


from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("omni.isaac.dynamic_control")  # Pegasus 5.1 still uses it
simulation_app.update()

sys.path.insert(0, PEGASUS_EXT)
sys.path.insert(0, os.path.join(HERE, ".deps"))  # pymavlink (Pegasus' PX4 backend), see README
sys.path.insert(0, RAPTOR_TESTS)
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api.objects import GroundPlane  # noqa: E402
from isaacsim.core.api.world import World  # noqa: E402
from pxr import UsdLux  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig  # noqa: E402
from pegasus.simulator.logic.dynamics import LinearDrag  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.sensors import GPS, IMU, Barometer, Magnetometer  # noqa: E402
from pegasus.simulator.logic.sensors.geo_mag_utils import EARTH_RADIUS  # noqa: E402
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.params import ROBOTS  # noqa: E402

from castor_assets import usd_build as U  # noqa: E402
from castor_assets.runtime import RigRuntime  # noqa: E402
from marl_scene import GoalMarker  # noqa: E402
from sim_loop import run_physics  # noqa: E402


def main():
    n = rig.num_drones
    rig.spawn.payload_clearance = 0.0  # payload on the ground
    vinfo = U.vehicle_info(rig, vehicle_cfg, os.path.join(C.GENERATED_DIR, "runs"), ROBOTS["Iris"], unique=True)
    taut_layout, warnings = U.rig_layout_for(rig, vinfo)
    for w in warnings:
        print(f"[px4] WARNING {w}")
    yaws = list(taut_layout.drone_yaw)
    ground = [np.array([p[0], p[1], vinfo.spawn_height_on_ground]) for p in taut_layout.drone_pos]
    mount_local = np.asarray(vinfo.mount_local, dtype=float)
    spawn_layout = dataclasses.replace(
        taut_layout, drone_pos=ground, mounts_world=[g + U.G.rot_z(y) @ mount_local for g, y in zip(ground, yaws)])

    px4 = Px4Instances(n, px4_parameters(vinfo))

    timeline = omni.timeline.get_timeline_interface()
    pg = PegasusInterface()
    pg._world = World(physics_dt=1.0 / PHYSICS_HZ, rendering_dt=1.0 / args.render_hz, stage_units_in_meters=1.0)
    world = pg.world
    GroundPlane(prim_path="/World/GroundPlane", size=100.0, color=np.array([0.4, 0.4, 0.45]))
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, "/World/Sun").CreateIntensityAttr(2500.0)
    UsdLux.DomeLight.Define(stage, "/World/Sky").CreateIntensityAttr(600.0)

    drone_paths = []
    for i in range(n):
        cfg = MultirotorConfig()
        cfg.thrust_curve = QuadraticThrustCurve(vinfo.thrust_curve)
        cfg.drag = LinearDrag(vinfo.linear_drag)
        cfg.sensors = [Barometer(), IMU(), Magnetometer(), GPS(RTK_GPS)]
        cfg.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig({
            "vehicle_id": i, "px4_autolaunch": False, "connection_type": "tcpin", "connection_ip": "localhost",
            "connection_baseport": 4560, "enable_lockstep": True, "num_rotors": 4, "update_rate": PHYSICS_HZ}))]
        path = f"/World/drone{i}"
        Multirotor(path, vinfo.usd_path, i, [float(x) for x in ground[i]],
                   [0.0, 0.0, math.sin(yaws[i] / 2), math.cos(yaws[i] / 2)], config=cfg)
        drone_paths.append(path)

    handles = U.author_rig(stage, rig, vinfo, spawn_layout, drone_paths)
    rig_rt = RigRuntime(stage, handles, rig, 1.0 / PHYSICS_HZ, time_fn=lambda: pg.world.current_time)
    marker = GoalMarker(stage, rig.payload.radius, rig.payload.height, n)

    world.reset()
    rig_rt.initialize()

    # The world frame every position is exchanged in: east-north-up about the simulated world's origin, whose
    # geodetic coordinates are the datum. PX4's GNSS fixes are relative to the same point.
    datum = [pg.latitude, pg.longitude, pg.altitude]
    # What the flight node needs to know about the rig; on an aircraft this is configuration.
    rig_info = {
        "trained_on": args.trained_on,
        "cable_length": rig.cable.length,
        "mount_local": mount_local.tolist(),
        "com_local": [float(x) for x in vinfo.summary["com"]],  # the body's centre of mass, body frame
        "anchors_local": [np.asarray(a).tolist() for a in taut_layout.anchors_local],
        # each drone's body origin relative to the payload centre when its cable is just taut and the payload level
        "slots": [(np.asarray(p) - taut_layout.payload_pos).tolist() for p in taut_layout.drone_pos],
        "yaws": [float(y) for y in yaws],
        "payload_height": rig.payload.height,
        "earth_radius": EARTH_RADIUS,  # the simulated GNSS receiver's; a real one is on the WGS-84 ellipsoid
    }
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    goal_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    goal_in.bind(("127.0.0.1", args.goal_port))
    goal_in.setblocking(False)
    rng = np.random.default_rng(0)
    every = max(1, int(round(PHYSICS_HZ / PAYLOAD_HZ)))
    state = {"step": 0, "goal": None}

    def on_physics_step(dt):
        state["step"] += 1
        if state["step"] % every:
            return
        position, quat = rig_rt._pose(rig_rt.payload_h)  # quaternion x y z w
        velocity = np.array(rig_rt.dc.get_rigid_body_linear_velocity(rig_rt.payload_h), dtype=float)
        k = args.payload_noise
        noisy = Rotation.from_rotvec(k * rng.normal(0.0, PAYLOAD_ROT_NOISE, 3)) * Rotation.from_quat(quat)
        q = noisy.as_quat()
        anchors, mounts = rig_rt.endpoints()
        message = {
            "t": world.current_time, "datum": datum, "rig": rig_info,
            "position": (position + k * rng.normal(0.0, PAYLOAD_POS_NOISE, 3)).tolist(),
            "orientation_wxyz": [q[3], q[0], q[1], q[2]],
            "velocity": velocity.tolist(),
            # for the operator's display only, never for control: the true rig state
            "truth": {"cable_span": np.linalg.norm(mounts - anchors, axis=1).round(4).tolist(),
                      "cable_length": rig.cable.length,
                      "drones": [np.round(rig_rt._pose(h)[0], 4).tolist() for h in rig_rt.drone_h]},
        }
        out.sendto(json.dumps(message).encode(), ("127.0.0.1", args.payload_port))

    world.add_physics_callback("castor_rig", rig_rt.on_physics_step)
    world.add_physics_callback("payload_out", on_physics_step)

    def on_frame():
        rig_rt.update_visuals()
        try:
            while True:
                state["goal"] = json.loads(goal_in.recv(4096).decode())
        except BlockingIOError:
            pass
        goal = state["goal"]
        if goal is not None:
            w, x, y, z = goal["orientation_wxyz"]
            marker.show(np.array(goal["position"]), Rotation.from_quat([x, y, z, w]).as_matrix(),
                        [np.array(p) for p in goal.get("setpoints", [])])

    centre = np.array([0.0, 0.0, 1.4])
    span = taut_layout.horizontal_distance + 1.0
    eye = centre + np.array([1.2 + 1.5 * span, -(1.2 + 1.5 * span), 0.6 + 0.8 * span])
    if not args.headless:
        from isaacsim.core.utils.viewports import set_camera_view

        set_camera_view(eye=eye, target=centre, camera_prim_path="/OmniverseKit_Persp")
        on_frame()

    print(f"[px4] {n} x S500 on the ground {taut_layout.horizontal_distance:.3f} m from the payload axis; "
          f"datum lat {datum[0]:.6f} lon {datum[1]:.6f} alt {datum[2]:.1f} m")
    print(f"[px4] payload pose -> udp 127.0.0.1:{args.payload_port}; goal marker <- udp 127.0.0.1:{args.goal_port}")
    print("[px4] PX4 uXRCE-DDS: " + ", ".join(f"drone {i}: udp {8888 + i} ns px4_{i + 1}" for i in range(n)))
    timeline.play()
    wall0, sim0 = time.perf_counter(), world.current_time

    def pace():
        """Headless there is no frame to wait for: keep simulated time from running ahead of the clock, because the
        DDS side of the loop lives in real time."""
        ahead = (world.current_time - sim0) - (time.perf_counter() - wall0)
        if ahead > 0:
            time.sleep(ahead)
        if state["step"] % every == 0:
            on_frame()  # receives the goal

    try:
        run_physics(world, simulation_app, args.duration or 1e9, PHYSICS_HZ, render=not args.headless,
                    render_hz=args.render_hz, realtime=not args.fast, on_frame=on_frame,
                    before_step=pace if args.headless and not args.fast else None)
    finally:
        timeline.stop()
        px4.stop()
        print("[px4] PX4 instances stopped")


if __name__ == "__main__":
    try:
        main()
    except Exception:  # Kit catches uncaught exceptions and still exits 0
        import traceback

        traceback.print_exc()
        os._exit(1)
    simulation_app.close()
