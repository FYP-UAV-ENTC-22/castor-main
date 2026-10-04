"""The simulator side of SIL: N S500s in Isaac Sim, each flown by its own PX4 SITL instance.

Run inside the simulation container after `sil.sh up` has started the onboard stacks:

    /isaac-sim/python.sh components/simulation/sil/sil_pegasus.py --headless --drones 3 --duration 60

Each vehicle i gets Pegasus' PX4 MAVLink backend (HIL over TCP 4560+i, lockstep) and a PX4 instance i started
by castor_px4 with the environment sil.sh wrote for it (ROS domain, XRCE agent port, no /fmu namespace), so
PX4 instance i talks only to robot i's stack. The vehicle is the S500 from components/simulation/assets
(mass, inertia, thrust curve, drag), not Pegasus' Iris. Prints the real-time factor at the end: in lockstep the
simulator, not the wall clock, sets PX4's pace, and the onboard stacks run on wall time, so the loop is held to
real time unless --fast; a factor below 1 means the simulator cannot keep up.
"""

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CASTOR_ROOT = os.environ.get("CASTOR_ROOT") or os.path.abspath(os.path.join(HERE, "../../.."))
PEGASUS_EXT = os.path.join(CASTOR_ROOT, "components/simulation/pegasus_simulator/extensions/pegasus.simulator")
ASSETS = os.path.join(CASTOR_ROOT, "components/simulation/assets")
PX4_DIR = os.path.join(CASTOR_ROOT, "components/vehicle/PX4-Autopilot")
sys.path.insert(0, ASSETS)

from castor_assets import config as C  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--drones", type=int, default=None, help="default: one per line of --px4-env")
parser.add_argument("--px4-env", default=os.path.join(CASTOR_ROOT, ".sil/px4.env"), help="written by sil.sh up")
parser.add_argument("--build", default="px4_sitl_default", choices=["px4_sitl_default", "px4_sitl_raptor"])
parser.add_argument("--airframe", default="gazebo-classic_iris", help="PX4 SITL model (MAVLink HIL airframe)")
parser.add_argument("--vehicle", default="s500.yaml", help="vehicle config in components/simulation/assets/config")
parser.add_argument("--vset", action="append", default=[], metavar="KEY=VALUE", help="vehicle override, repeatable")
parser.add_argument("--spacing", type=float, default=2.5, help="m between drones along y")
parser.add_argument("--duration", type=float, default=60.0, help="simulated seconds; 0 = until the window closes")
parser.add_argument("--fast", action="store_true",
                    help="do not hold the simulation to real time (the onboard stacks run on wall time, so SIL needs it)")
args = parser.parse_args()

vehicle_cfg = C.load_vehicle(args.vehicle, args.vset)  # config mistakes fail before Isaac Sim starts
sys.stdout.reconfigure(line_buffering=True)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("omni.isaac.dynamic_control")  # Pegasus 5.1 still uses it
simulation_app.update()
sys.path.insert(0, PEGASUS_EXT)
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api.objects import GroundPlane  # noqa: E402
from isaacsim.core.api.world import World  # noqa: E402
from pxr import UsdLux  # noqa: E402

from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig  # noqa: E402
from pegasus.simulator.logic.dynamics import LinearDrag  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.params import ROBOTS  # noqa: E402

import castor_px4  # noqa: E402
from castor_assets import usd_build as U  # noqa: E402


def main():
    instances = castor_px4.read_env_file(args.px4_env) if os.path.exists(args.px4_env) else {}
    n = args.drones or len(instances)
    if n < 1:
        raise SystemExit(f"no drones: pass --drones or run `sil.sh up` first ({args.px4_env} missing)")
    missing = [i for i in range(n) if i not in instances]
    if missing:
        print(f"[sil] WARNING no SIL environment for PX4 instance(s) {missing}: PX4 defaults (domain 0, port 8888)")
    castor_px4.install(px4_env_file=args.px4_env if instances else None, build=args.build)

    rig = C.load_rig("payload_rig.yaml", [f"vehicle_config={args.vehicle}", "num_drones=1"])
    vinfo = U.vehicle_info(rig, vehicle_cfg, os.path.join(C.GENERATED_DIR, "runs"), ROBOTS["Iris"], unique=True)
    s = vinfo.summary
    print(f"[sil] S500 {s['total_mass']:.3f} kg, T/W {s['thrust_to_weight']:.2f}, USD {vinfo.usd_path}")

    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    GroundPlane(prim_path="/World/GroundPlane", size=100.0, color=np.array([0.4, 0.4, 0.45]))
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, "/World/Sun").CreateIntensityAttr(2500.0)
    UsdLux.DomeLight.Define(stage, "/World/Sky").CreateIntensityAttr(600.0)

    z0 = vinfo.spawn_height_on_ground  # skids on the ground, as PX4 expects at boot
    for i in range(n):
        cfg = MultirotorConfig()
        cfg.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig({
            "vehicle_id": i, "px4_autolaunch": True, "px4_dir": PX4_DIR, "px4_vehicle_model": args.airframe,
        }))]
        if vinfo.thrust_curve:
            cfg.thrust_curve = QuadraticThrustCurve(vinfo.thrust_curve)
            cfg.drag = LinearDrag(vinfo.linear_drag)
        Multirotor(f"/World/drone{i}", vinfo.usd_path, i, [0.0, args.spacing * i, z0], [0.0, 0.0, 0.0, 1.0],
                   config=cfg)
        env = instances.get(i, {})
        print(f"[sil] drone{i + 1}: PX4 instance {i} ({args.build}), HIL tcp 4560+{i}, "
              f"domain {env.get('ROS_DOMAIN_ID', '?')}, agent udp {env.get('PX4_UXRCE_DDS_PORT', '8888')}")

    world.reset()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    wall0, steps = time.perf_counter(), 0
    while simulation_app.is_running() and (args.duration <= 0 or world.current_time < args.duration):
        world.step(render=not args.headless)
        steps += 1
        ahead = world.current_time - (time.perf_counter() - wall0)
        if not args.fast and ahead > 0.002:
            time.sleep(ahead)
        if steps % 2500 == 0:
            wall = time.perf_counter() - wall0
            print(f"[sil] sim {world.current_time:7.1f} s, wall {wall:7.1f} s, real-time factor "
                  f"{world.current_time / wall:.2f}")
    wall = time.perf_counter() - wall0
    print(f"[sil] done: {world.current_time:.1f} s simulated in {wall:.1f} s wall, {steps} steps, "
          f"real-time factor {world.current_time / max(wall, 1e-9):.2f}")
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback

        traceback.print_exc()
        sys.stdout.flush()
        os._exit(1)
