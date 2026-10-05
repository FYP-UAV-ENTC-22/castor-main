"""The simulator side of SIL: the configured payload rig in Isaac Sim, every drone flown by its own PX4 SITL instance.

    make sim-pegasus-ros2                     # GUI, until the window closes; HEADLESS=1, DRONES=N, DURATION=s
    /isaac-sim/python.sh components/simulation/sil/sil_pegasus.py --headless --duration 60

The rig comes from components/simulation/assets/config/payload_rig.yaml (number of drones, payload, cables, formation)
and its vehicle config (s500.yaml); override keys with --set / --vset. Everything starts on the ground, as PX4 expects
at boot: each drone on its skids at its formation x, y and yaw, the payload resting on the ground, the cables slack.
Nothing arms or takes off; the drones wait for commands from their stacks.

Each vehicle i gets Pegasus' PX4 MAVLink backend (HIL over TCP 4560+i, lockstep) and a PX4 instance i started by
castor_px4 with the environment `sil.sh up` wrote for robot i+1 (ROS domain 21+i, XRCE agent UDP 8888+i, no /fmu
namespace), so PX4 instance i talks only to robot i+1's stack. Without .sil/px4.env the same scheme is used, so the
stacks can be started before or after the simulator. Isaac ground truth goes to the simulator's own domain (20):
sim/drone<i+1>/state/{pose,twist,twist_inertial,accel} and sim/payload/state/{pose,twist_inertial} (ENU, frame map),
at --ground-truth-hz (50) of simulated time. Robot i+1's domain gets its own drone's and the payload's
(/sim/drone<i+1>/state/{pose,twist,twist_inertial}, /sim/payload/state/pose): until localization exists, the planning
policy reads the world frame from there.

PX4 runs the RAPTOR build by default (--build px4_sitl_default for stock PX4), with MC_RAPTOR_ENABLE=1, MC_RAPTOR_OFFB=0
(a separate external mode that holds position by itself and follows trajectory_setpoint while it is fresh),
MC_RAPTOR_INTREF=0, IMU_GYRO_RATEMAX=250 and NAV_DLL_ACT=0, set through PX4_PARAM_* for SITL only.

The loop is held to real time unless --fast: in lockstep the simulator sets PX4's pace, and the onboard stacks run on
wall time. The real-time factor is printed every 2500 steps; below 1 the simulator cannot keep up.
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
parser.add_argument("--drones", type=int, default=None, help="default: the rig's num_drones")
parser.add_argument("--rig", default="payload_rig.yaml", help="rig config (path, or a name in assets/config/)")
parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="rig override, repeatable")
parser.add_argument("--vehicle", default=None, help="vehicle config; default: the rig's vehicle_config")
parser.add_argument("--vset", action="append", default=[], metavar="KEY=VALUE", help="vehicle override, repeatable")
parser.add_argument("--no-payload", action="store_true", help="the drones in their formation, no payload or cables")
parser.add_argument("--no-ground-truth", action="store_true", help="do not publish Isaac ground truth on sim/*")
parser.add_argument("--ground-truth-hz", type=float, default=50.0,
                    help="sim/* rate in simulated time; every message is published from Python, so it costs real time")
parser.add_argument("--px4-env", default=os.path.join(CASTOR_ROOT, ".sil/px4.env"), help="written by sil.sh up")
parser.add_argument("--build", default="px4_sitl_raptor", choices=["px4_sitl_default", "px4_sitl_raptor"])
parser.add_argument("--airframe", default="gazebo-classic_iris", help="PX4 SITL model (MAVLink HIL airframe)")
parser.add_argument("--duration", type=float, default=60.0, help="simulated seconds; 0 = until the window closes")
parser.add_argument("--fast", action="store_true",
                    help="do not hold the simulation to real time (the onboard stacks run on wall time, so SIL needs it)")
args = parser.parse_args()

# config mistakes fail here, before Isaac Sim starts
rig = C.load_rig(args.rig, args.set + ([f"num_drones={args.drones}"] if args.drones else []))
vehicle_cfg = C.load_vehicle(args.vehicle or rig.vehicle_config, args.vset)
if not args.no_payload and rig.cable.model != "distance":
    parser.error(f"the rig starts on the ground with slack cables, which needs cable.model=distance (config has "
                 f"{rig.cable.model!r}); pass --set cable.model=distance or --no-payload")
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
from castor_assets.runtime import RigRuntime  # noqa: E402


def ground_truth_every(physics_dt):
    return max(1, round(1.0 / (args.ground_truth_hz * physics_dt)))


class PayloadGroundTruth:
    """Publishes the payload's Isaac pose and inertial twist (ENU, frame map) on the simulator's domain."""

    def __init__(self, rig_rt, every):
        import rclpy
        from geometry_msgs.msg import PoseStamped, TwistStamped

        try:
            rclpy.init()
        except RuntimeError:
            pass  # Pegasus' ROS2Backend already did
        self.rig_rt, self.Pose, self.Twist = rig_rt, PoseStamped, TwistStamped
        self.every, self.calls = every, 0
        self.node = rclpy.create_node("simulator_payload")
        qos = rclpy.qos.qos_profile_sensor_data
        self.pose_pub = self.node.create_publisher(PoseStamped, "sim/payload/state/pose", qos)
        self.twist_pub = self.node.create_publisher(TwistStamped, "sim/payload/state/twist_inertial", qos)

    def publish(self, _dt):
        self.calls += 1
        if self.calls % self.every:
            return
        dc, h = self.rig_rt.dc, self.rig_rt.payload_h
        p = dc.get_rigid_body_pose(h)
        v = dc.get_rigid_body_linear_velocity(h)
        w = dc.get_rigid_body_angular_velocity(h)
        stamp = self.node.get_clock().now().to_msg()
        pose = self.Pose()
        pose.header.stamp, pose.header.frame_id = stamp, "map"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (float(x) for x in p.p)
        o = pose.pose.orientation
        o.x, o.y, o.z, o.w = (float(x) for x in p.r)
        twist = self.Twist()
        twist.header.stamp, twist.header.frame_id = stamp, "map"
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = (float(x) for x in v)
        twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = (float(x) for x in w)
        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)


class RobotDomainGroundTruth:
    """Each drone's own state and the payload pose, published into that robot's ROS domain (its stack's world frame).

    One rclpy context per domain: the simulator process sits on domain 20, each SIL robot on its own."""

    def __init__(self, vehicles, domains, rig_rt, every):
        import rclpy
        from geometry_msgs.msg import PoseStamped, TwistStamped

        self.rclpy, self.Pose, self.Twist = rclpy, PoseStamped, TwistStamped
        self.vehicles, self.rig_rt, self.every, self.calls = vehicles, rig_rt, every, 0
        qos = rclpy.qos.qos_profile_sensor_data
        self.robots = []
        for i, domain in enumerate(domains):
            ctx = rclpy.context.Context()
            rclpy.init(context=ctx, domain_id=domain)
            node = rclpy.create_node(f"simulator_drone{i + 1}", context=ctx)
            ns = f"/sim/drone{i + 1}/state"
            self.robots.append((ctx, node, {
                "pose": node.create_publisher(PoseStamped, f"{ns}/pose", qos),
                "twist": node.create_publisher(TwistStamped, f"{ns}/twist", qos),
                "twist_inertial": node.create_publisher(TwistStamped, f"{ns}/twist_inertial", qos),
                "payload": node.create_publisher(PoseStamped, "/sim/payload/state/pose", qos)
                if rig_rt is not None else None,
            }))

    def _pose(self, stamp, p, q):
        m = self.Pose()
        m.header.stamp, m.header.frame_id = stamp, "map"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(x) for x in p)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (float(x) for x in q)
        return m

    def _twist(self, stamp, frame, v, w):
        m = self.Twist()
        m.header.stamp, m.header.frame_id = stamp, frame
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (float(x) for x in v)
        m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = (float(x) for x in w)
        return m

    def publish(self, _dt):
        self.calls += 1
        if self.calls % self.every:
            return
        payload = None
        if self.rig_rt is not None:
            pp = self.rig_rt.dc.get_rigid_body_pose(self.rig_rt.payload_h)
            payload = (pp.p, pp.r)  # dynamic_control quaternion is x, y, z, w
        for i, (_ctx, node, pubs) in enumerate(self.robots):
            st = self.vehicles[i].state
            stamp = node.get_clock().now().to_msg()
            pubs["pose"].publish(self._pose(stamp, st.position, st.attitude))
            # Pegasus' convention: body-frame linear and angular velocity; inertial linear velocity.
            pubs["twist"].publish(self._twist(stamp, f"drone{i + 1}/base_link", st.linear_body_velocity,
                                              st.angular_velocity))
            pubs["twist_inertial"].publish(self._twist(stamp, "map", st.linear_velocity, (0.0, 0.0, 0.0)))
            if payload is not None:
                pubs["payload"].publish(self._pose(stamp, *payload))

    def close(self):
        for ctx, node, _ in self.robots:
            node.destroy_node()
            self.rclpy.shutdown(context=ctx)


def px4_instances(n):
    """Environment per PX4 instance: .sil/px4.env where it has one, otherwise the scheme sil.sh uses."""
    instances = castor_px4.read_env_file(args.px4_env) if os.path.exists(args.px4_env) else {}
    if not instances:
        print(f"[sil] {args.px4_env} not found: PX4 instance i uses sil.sh's scheme (domain 21+i, agent udp 8888+i); "
              f"start the stacks with `components/simulation/sil/sil.sh up --drones {n}`")
    elif len(instances) != n:
        print(f"[sil] WARNING {args.px4_env} has {len(instances)} instance(s) for {n} drones: run "
              f"`sil.sh up --drones {n}` so every drone has a stack; the rest use sil.sh's scheme with no stack behind it")
    return {i: instances.get(i, castor_px4.sil_env(i)) for i in range(n)}


def main():
    n = rig.num_drones
    instances = px4_instances(n)
    raptor_params = {"PX4_PARAM_MC_RAPTOR_ENABLE": "1", "PX4_PARAM_MC_RAPTOR_OFFB": "0",
                     "PX4_PARAM_MC_RAPTOR_INTREF": "0", "PX4_PARAM_IMU_GYRO_RATEMAX": "250",
                     "PX4_PARAM_NAV_DLL_ACT": "0"}
    castor_px4.install(build=args.build, instances=instances,
                       extra_env=raptor_params if args.build == "px4_sitl_raptor" else None)

    vinfo = U.vehicle_info(rig, vehicle_cfg, os.path.join(C.GENERATED_DIR, "runs"), ROBOTS["Iris"], unique=True)
    layout, warnings = U.rig_layout_for(rig, vinfo)
    for w in warnings:
        print(f"[sil] WARNING {w}")
    layout = U.grounded_layout(rig, vinfo, layout, cables=not args.no_payload)
    if vinfo.kind == "s500":
        s = vinfo.summary
        print(f"[sil] S500 {s['total_mass']:.3f} kg, T/W {s['thrust_to_weight']:.2f}, USD {vinfo.usd_path}")
    print(f"[sil] rig {args.rig}: {n} x {rig.vehicle}"
          + ("" if args.no_payload else f", payload {rig.payload.mass} kg, {rig.cable.model} cables {rig.cable.length} m")
          + f", drones {layout.horizontal_distance:.3f} m from the payload axis, all on the ground")

    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    GroundPlane(prim_path="/World/GroundPlane", size=100.0, color=np.array([0.4, 0.4, 0.45]))
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, "/World/Sun").CreateIntensityAttr(2500.0)
    UsdLux.DomeLight.Define(stage, "/World/Sky").CreateIntensityAttr(600.0)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    ground_truth = not args.no_ground_truth
    gt_every = ground_truth_every(world.get_physics_dt())
    if ground_truth:
        from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend

        class GroundTruthBackend(ROS2Backend):
            """Pegasus' ROS2Backend, publishing state only, every gt_every-th physics step."""

            calls = 0

            def update_state(self, state):
                self.calls += 1
                if self.calls % gt_every == 0:
                    super().update_state(state)

            def update(self, dt):
                pass  # no subscriptions (sub_control off), so nothing to spin

    drone_paths, vehicles = [], []
    for i in range(n):
        cfg = MultirotorConfig()
        # backends[0] drives the rotors (Multirotor reads its input_reference): PX4 first, ground truth after
        cfg.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig({
            "vehicle_id": i, "px4_autolaunch": True, "px4_dir": PX4_DIR, "px4_vehicle_model": args.airframe,
        }))]
        if ground_truth:
            cfg.backends.append(GroundTruthBackend(vehicle_id=i + 1, config={
                "namespace": "sim/drone", "pub_sensors": False, "pub_graphical_sensors": False, "sub_control": False}))
        if vinfo.thrust_curve:
            cfg.thrust_curve = QuadraticThrustCurve(vinfo.thrust_curve)
            cfg.drag = LinearDrag(vinfo.linear_drag)
        yaw = layout.drone_yaw[i]
        path = f"/World/drone{i}"
        vehicles.append(Multirotor(path, vinfo.usd_path, i, [float(x) for x in layout.drone_pos[i]],
                                   [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)], config=cfg))
        drone_paths.append(path)
        env = instances[i]
        print(f"[sil] drone{i + 1}: PX4 instance {i} ({args.build}), HIL tcp {4560 + i}, "
              f"domain {env.get('ROS_DOMAIN_ID', '?')}, agent udp {env.get('PX4_UXRCE_DDS_PORT', '8888')}")

    rig_rt = None
    if not args.no_payload:
        handles = U.author_rig(stage, rig, vinfo, layout, drone_paths)
        rig_rt = RigRuntime(stage, handles, rig, world.get_physics_dt(), time_fn=lambda: world.current_time)

    world.reset()
    if rig_rt is not None:
        rig_rt.initialize()  # only for drawing the cables and reading the payload; it logs nothing here
        if ground_truth:
            world.add_physics_callback("castor_payload_ground_truth", PayloadGroundTruth(rig_rt, gt_every).publish)
    robot_gt = None
    if ground_truth:
        print(f"[sil] ground truth at {1.0 / (gt_every * world.get_physics_dt()):.0f} Hz (simulated) on domain "
              f"{os.environ.get('ROS_DOMAIN_ID', '0')}: sim/drone<i>/state/*"
              + ("" if rig_rt is None else ", sim/payload/state/*"))
        domains = [int(instances[i].get("ROS_DOMAIN_ID", 21 + i)) for i in range(n)]
        robot_gt = RobotDomainGroundTruth(vehicles, domains, rig_rt, gt_every)
        world.add_physics_callback("castor_robot_ground_truth", robot_gt.publish)
        print(f"[sil] and into each robot's domain {domains}: /sim/drone<i>/state/{{pose,twist,twist_inertial}}"
              + ("" if rig_rt is None else ", /sim/payload/state/pose"))

    if not args.headless:
        from isaacsim.core.utils.viewports import set_camera_view

        centre = layout.payload_pos + np.array([0.0, 0.0, 0.5])
        span = max(layout.horizontal_distance, 0.5)
        set_camera_view(eye=centre + np.array([1.5 + 1.6 * span, -(1.5 + 1.6 * span), 1.0 + 0.9 * span]),
                        target=centre, camera_prim_path="/OmniverseKit_Persp")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    print("[sil] running: PX4 boots disarmed; nothing arms or takes off until a stack commands it")
    wall0, steps = time.perf_counter(), 0
    try:
        while simulation_app.is_running() and (args.duration <= 0 or world.current_time < args.duration):
            if rig_rt is not None and not args.headless:
                rig_rt.update_visuals()
            world.step(render=not args.headless)
            steps += 1
            ahead = world.current_time - (time.perf_counter() - wall0)
            if not args.fast and ahead > 0.002:
                time.sleep(ahead)
            if steps % 2500 == 0:
                wall = time.perf_counter() - wall0
                print(f"[sil] sim {world.current_time:7.1f} s, wall {wall:7.1f} s, real-time factor "
                      f"{world.current_time / wall:.2f}")
    except KeyboardInterrupt:
        print("[sil] interrupted")
    finally:
        timeline.stop()  # Pegasus' PX4 backends kill their PX4 instances on stop
        if robot_gt is not None:
            robot_gt.close()
    wall = time.perf_counter() - wall0
    print(f"[sil] done: {world.current_time:.1f} s simulated in {wall:.1f} s wall, {steps} steps, "
          f"real-time factor {world.current_time / max(wall, 1e-9):.2f}")
    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback

        traceback.print_exc()
        sys.stdout.flush()
        os._exit(1)
