"""Isaac Sim + Pegasus publishing into the ROS 2 graph: one S500 per --drones, Pegasus' ROS2Backend, no PX4.

Run in the simulation container (headless or with make sim-gui's display):

    /isaac-sim/python.sh components/simulation/tests/pegasus_ros2.py --headless --duration 60

Each vehicle i publishes drone<i>/state/{pose,twist,twist_inertial,accel} and drone<i>/sensors/{imu,mag,gps,
gps_twist,baro} (sensor-data QoS) through the ROS 2 Jazzy bundled with Isaac's ROS bridge, on ROS_DOMAIN_ID
(20) with docker/fastdds_localhost.xml, i.e. into the same graph as the onboard containers. Nothing drives the
rotors, so the vehicles sit on the ground; the point is the plumbing. See docker/README.md for seeing it from the
host's own ROS install.
"""

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CASTOR_ROOT = os.environ.get("CASTOR_ROOT") or os.path.abspath(os.path.join(HERE, "../../.."))
PEGASUS_EXT = os.path.join(CASTOR_ROOT, "components/simulation/pegasus_simulator/extensions/pegasus.simulator")
sys.path.insert(0, os.path.join(CASTOR_ROOT, "components/simulation/assets"))

from castor_assets import config as C  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--drones", type=int, default=1)
parser.add_argument("--duration", type=float, default=60.0, help="seconds of simulated time; 0 = until closed")
args = parser.parse_args()
vehicle_cfg = C.load_vehicle("s500.yaml")
sys.stdout.reconfigure(line_buffering=True)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("omni.isaac.dynamic_control")  # Pegasus 5.1 still uses it
simulation_app.update()
sys.path.insert(0, PEGASUS_EXT)

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api.objects import GroundPlane  # noqa: E402
from isaacsim.core.api.world import World  # noqa: E402
from pxr import UsdLux  # noqa: E402

from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.params import ROBOTS  # noqa: E402

from castor_assets import usd_build as U  # noqa: E402


def main():
    rig = C.load_rig("payload_rig.yaml", ["num_drones=1"])
    vinfo = U.vehicle_info(rig, vehicle_cfg, os.path.join(C.GENERATED_DIR, "runs"), ROBOTS["Iris"], unique=True)
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    GroundPlane(prim_path="/World/GroundPlane", size=100.0, color=np.array([0.4, 0.4, 0.45]))
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, "/World/Sun").CreateIntensityAttr(2500.0)
    UsdLux.DomeLight.Define(stage, "/World/Sky").CreateIntensityAttr(600.0)
    for i in range(args.drones):
        cfg = MultirotorConfig()
        cfg.backends = [ROS2Backend(vehicle_id=i, config={"namespace": "drone", "pub_graphical_sensors": False})]
        Multirotor(f"/World/drone{i}", vinfo.usd_path, i, [0.0, 2.5 * i, vinfo.spawn_height_on_ground],
                   [0.0, 0.0, 0.0, 1.0], config=cfg)
    print(f"[pegasus_ros2] {args.drones} x S500, ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}, "
          f"topics drone<i>/state/*, drone<i>/sensors/*")
    world.reset()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    wall0 = time.perf_counter()
    while simulation_app.is_running() and (args.duration <= 0 or world.current_time < args.duration):
        world.step(render=not args.headless)
        ahead = world.current_time - (time.perf_counter() - wall0)
        if ahead > 0.002:
            time.sleep(ahead)  # real time, so subscribers see real rates
    print(f"[pegasus_ros2] done, {world.current_time:.1f} s simulated")
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
