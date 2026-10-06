"""The simulator side of stack_sim: the model's payload rig in Isaac Sim, every drone flown by its own PX4 SITL instance.

    make sim-pegasus-ros2                     # GUI, until the window closes; HEADLESS=1, MODEL=, DURATION=s
    /isaac-sim/python.sh components/simulation/stack_sim/stack_sim_pegasus.py --headless --duration 60

Everything comes from the assets (rig_check.py): the model package (--model, default models/DEFAULT) names the rig it
was trained on (model.yaml rig.config), which is loaded from components/simulation/assets/config with its vehicle file;
the model's other rig numbers are checked against it and a mismatch stops the run. --rig picks another rig file (it
must still match the model), --set / --vset override keys. Everything starts on the ground, as PX4 expects at boot:
each drone on its skids at its formation x, y and yaw, the payload resting on the ground, the cables slack. Nothing
arms or takes off; the drones wait for commands from their stacks.

Each vehicle i gets Pegasus' PX4 MAVLink backend (HIL over TCP 4560+i, lockstep) and a PX4 instance i started by
castor_px4 with the environment `stack_sim.sh up` wrote for robot i+1 (ROS domain 21+i, XRCE agent UDP 8888+i, no /fmu
namespace), so PX4 instance i talks only to robot i+1's stack. Without .stack_sim/px4.env the same scheme is used, so
the stacks can be started before or after the simulator. Isaac ground truth goes to the simulator's own domain (20):
sim/drone<i+1>/state/{pose,twist,twist_inertial,accel} and sim/payload/state/{pose,twist_inertial} (ENU, frame map),
at --ground-truth-hz (50) of simulated time. Robot i+1's domain gets its own drone's and the payload's
(/sim/drone<i+1>/state/{pose,twist,twist_inertial}, /sim/payload/state/pose), stamped with simulated time, and /clock
at the same instants: until localization exists the planning policy reads the world frame from there and steps once
per payload sample, and the mission node times its states on /clock.

PX4 runs the RAPTOR build by default (--build px4_sitl_default for stock PX4) on the none_iris airframe, every instance
from fresh parameters set through PX4_PARAM_* for SITL only: assets/config/px4_sitl.yaml (RAPTOR as its own mode, no
RC or ground station, no auto-disarm, the GNSS receiver and EKF2 height source, no uXRCE-DDS time sync) plus what the
vehicle file decides (rotor positions and yaw moments, hover throttle with the drone's share of the payload,
IMU_GYRO_RATEMAX = the physics rate).

The loop is held to real time unless --fast: in lockstep the simulator sets PX4's pace, and the onboard stacks' links
and timeouts run on wall time. The real-time factor is printed every 2500 steps; below 1 the simulator cannot keep up.
"""

import argparse
import math
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CASTOR_ROOT = os.environ.get("CASTOR_ROOT") or os.path.abspath(os.path.join(HERE, "../../.."))
PEGASUS_EXT = os.path.join(CASTOR_ROOT, "components/simulation/pegasus_simulator/extensions/pegasus.simulator")
ASSETS = os.path.join(CASTOR_ROOT, "components/simulation/assets")
PX4_DIR = os.path.join(CASTOR_ROOT, "components/vehicle/PX4-Autopilot")
sys.path.insert(0, ASSETS)

from castor_assets import config as C  # noqa: E402

sys.path.insert(0, HERE)
import rig_check as R  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--drones", type=int, default=None, help="default: the rig's num_drones")
parser.add_argument("--model", default=None, help="model package (<name>/<version> or a dir); default models/DEFAULT")
parser.add_argument("--rig", default=None, help="rig config instead of the model's (path, or a name in assets/config/)")
parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="rig override, repeatable")
parser.add_argument("--vehicle", default=None, help="vehicle config; default: the rig's vehicle_config")
parser.add_argument("--vset", action="append", default=[], metavar="KEY=VALUE", help="vehicle override, repeatable")
parser.add_argument("--no-payload", action="store_true", help="the drones in their formation, no payload or cables")
parser.add_argument("--no-ground-truth", action="store_true", help="do not publish Isaac ground truth on sim/*")
parser.add_argument("--ground-truth-hz", type=float, default=50.0,
                    help="sim/* rate in simulated time; every message is published from Python, so it costs real time")
parser.add_argument("--px4-env", default=os.path.join(CASTOR_ROOT, ".stack_sim/px4.env"),
                    help="written by stack_sim.sh up")
parser.add_argument("--build", default="px4_sitl_raptor", choices=["px4_sitl_default", "px4_sitl_raptor"])
parser.add_argument("--airframe", default="none_iris", help="PX4 SITL model (MAVLink HIL airframe)")
parser.add_argument("--keep-params", action="store_true", help="keep each PX4 instance's saved parameters")
parser.add_argument("--duration", type=float, default=60.0, help="simulated seconds; 0 = until the window closes")
parser.add_argument("--fast", action="store_true",
                    help="do not hold the simulation to real time (the stacks' links and timeouts run on wall time)")
args = parser.parse_args()

# config mistakes fail here, before Isaac Sim starts
try:
    plan = R.plan(args.model, args.rig, args.set + ([f"num_drones={args.drones}"] if args.drones else []), args.vset)
except (R.RigError, C.ConfigError, OSError) as e:
    parser.error(str(e))
if plan.mismatches:
    parser.error(f"the rig in the assets is not the rig {plan.model_id} was trained on: " + "; ".join(plan.mismatches))
rig = plan.rig
vehicle_cfg = C.load_vehicle(args.vehicle, args.vset) if args.vehicle else plan.vehicle
if not args.no_payload and rig.cable.model != "distance":
    parser.error(f"the rig starts on the ground with slack cables, which needs cable.model=distance (config has "
                 f"{rig.cable.model!r}); pass --set cable.model=distance or --no-payload")
# A second simulator cannot get the HIL ports. Pegasus only logs that, and the two would then publish ground truth
# and /clock into the same robot domains, so the stacks fly an invisible run on a clock that jumps between the two.
for i in range(rig.num_drones):
    with socket.socket() as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # as pymavlink binds; TIME_WAIT is no conflict
        try:
            s.bind(("127.0.0.1", 4560 + i))
        except OSError:
            parser.error(f"tcp {4560 + i} (PX4 HIL for drone{i + 1}) is taken: another simulator is running. "
                         "Stop it first (check `ps` in the simulation container)")
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

from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig  # noqa: E402
from pegasus.simulator.logic.dynamics import LinearDrag  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.sensors import GPS, IMU, Barometer, Magnetometer  # noqa: E402
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.params import ROBOTS  # noqa: E402

import castor_px4  # noqa: E402
from castor_assets import usd_build as U  # noqa: E402
from castor_assets.runtime import RigRuntime  # noqa: E402


PHYSICS_HZ = 250  # Pegasus' world; PX4's IMU_GYRO_RATEMAX follows it (mc_raptor drops out on gyro older than 10 ms)


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
    """Each drone's own state and the payload pose, published into that robot's ROS domain (its stack's world frame),
    stamped with simulated time, then /clock for the same instant (the stacks' use_sim_time).

    One rclpy context per domain: the simulator process sits on domain 20, each stack_sim robot on its own."""

    def __init__(self, vehicles, domains, rig_rt, every):
        import rclpy
        from builtin_interfaces.msg import Time
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from rosgraph_msgs.msg import Clock

        self.rclpy, self.Pose, self.Twist, self.Time, self.Clock = rclpy, PoseStamped, TwistStamped, Time, Clock
        self.vehicles, self.rig_rt, self.every, self.calls = vehicles, rig_rt, every, 0
        self.sim_time = 0.0
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
                "clock": node.create_publisher(Clock, "/clock", 10),
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

    def publish(self, dt):
        self.calls += 1
        self.sim_time += dt
        if self.calls % self.every:
            return
        ns = round(self.sim_time * 1e9)
        stamp = self.Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
        clock = self.Clock(clock=stamp)
        payload = None
        if self.rig_rt is not None:
            pp = self.rig_rt.dc.get_rigid_body_pose(self.rig_rt.payload_h)
            payload = (pp.p, pp.r)  # dynamic_control quaternion is x, y, z, w
        for i, (_ctx, node, pubs) in enumerate(self.robots):
            st = self.vehicles[i].state
            pubs["pose"].publish(self._pose(stamp, st.position, st.attitude))
            # Pegasus' convention: body-frame linear and angular velocity; inertial linear velocity.
            pubs["twist"].publish(self._twist(stamp, f"drone{i + 1}/base_link", st.linear_body_velocity,
                                              st.angular_velocity))
            pubs["twist_inertial"].publish(self._twist(stamp, "map", st.linear_velocity, (0.0, 0.0, 0.0)))
            if payload is not None:
                pubs["payload"].publish(self._pose(stamp, *payload))
            pubs["clock"].publish(clock)

    def close(self):
        for ctx, node, _ in self.robots:
            node.destroy_node()
            self.rclpy.shutdown(context=ctx)


def px4_instances(n):
    """Environment per PX4 instance: .stack_sim/px4.env where it has one, otherwise the scheme stack_sim.sh uses."""
    instances = castor_px4.read_env_file(args.px4_env) if os.path.exists(args.px4_env) else {}
    if not instances:
        print(f"[stack_sim] {args.px4_env} not found: PX4 instance i uses stack_sim.sh's scheme (domain 21+i, agent udp "
              f"8888+i); start the stacks with `components/simulation/stack_sim/stack_sim.sh up`")
    elif len(instances) != n:
        print(f"[stack_sim] WARNING {args.px4_env} has {len(instances)} instance(s) for {n} drones: run "
              f"`stack_sim.sh up` with the same model so every drone has a stack; the rest use stack_sim.sh's scheme with "
              f"no stack behind it")
    return {i: instances.get(i, castor_px4.stack_env(i)) for i in range(n)}


def main():
    n = rig.num_drones
    instances = px4_instances(n)
    vinfo = U.vehicle_info(rig, vehicle_cfg, os.path.join(C.GENERATED_DIR, "runs"), ROBOTS["Iris"], unique=True)
    px4_params = R.px4_parameters(plan, args.build, PHYSICS_HZ, vinfo.total_mass, payload=not args.no_payload)
    castor_px4.install(build=args.build, instances=instances, extra_env=px4_params, fresh_params=not args.keep_params)
    print("[stack_sim] PX4 parameters: " + ", ".join(f"{k[len('PX4_PARAM_'):]}={v}" for k, v in px4_params.items()))

    layout, warnings = U.rig_layout_for(rig, vinfo)
    for w in warnings:
        print(f"[stack_sim] WARNING {w}")
    layout = U.grounded_layout(rig, vinfo, layout, cables=not args.no_payload)
    if vinfo.kind == "s500":
        s = vinfo.summary
        print(f"[stack_sim] S500 {s['total_mass']:.3f} kg, T/W {s['thrust_to_weight']:.2f}, USD {vinfo.usd_path}")
    print(f"[stack_sim] model {plan.model_id}, rig {plan.rig_file}: {n} x {rig.vehicle}"
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

    gps = R.gps_sensor(plan)  # None: Pegasus' default receiver
    drone_paths, vehicles = [], []
    for i in range(n):
        cfg = MultirotorConfig()
        if gps is not None:
            cfg.sensors = [Barometer(), IMU(), Magnetometer(), GPS(gps)]
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
        print(f"[stack_sim] drone{i + 1}: PX4 instance {i} ({args.build}), HIL tcp {4560 + i}, "
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
        print(f"[stack_sim] ground truth at {1.0 / (gt_every * world.get_physics_dt()):.0f} Hz (simulated) on domain "
              f"{os.environ.get('ROS_DOMAIN_ID', '0')}: sim/drone<i>/state/*"
              + ("" if rig_rt is None else ", sim/payload/state/*"))
        domains = [int(instances[i].get("ROS_DOMAIN_ID", 21 + i)) for i in range(n)]
        robot_gt = RobotDomainGroundTruth(vehicles, domains, rig_rt, gt_every)
        world.add_physics_callback("castor_robot_ground_truth", robot_gt.publish)
        print(f"[stack_sim] and into each robot's domain {domains}: /sim/drone<i>/state/{{pose,twist,twist_inertial}}"
              + ("" if rig_rt is None else ", /sim/payload/state/pose"))

    if not args.headless:
        from isaacsim.core.utils.viewports import set_camera_view

        centre = layout.payload_pos + np.array([0.0, 0.0, 0.5])
        span = max(layout.horizontal_distance, 0.5)
        set_camera_view(eye=centre + np.array([1.5 + 1.6 * span, -(1.5 + 1.6 * span), 1.0 + 0.9 * span]),
                        target=centre, camera_prim_path="/OmniverseKit_Persp")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    print("[stack_sim] running: PX4 boots disarmed; nothing arms or takes off until a stack commands it")
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
                print(f"[stack_sim] sim {world.current_time:7.1f} s, wall {wall:7.1f} s, real-time factor "
                      f"{world.current_time / wall:.2f}")
    except KeyboardInterrupt:
        print("[stack_sim] interrupted")
    finally:
        timeline.stop()  # Pegasus' PX4 backends kill their PX4 instances on stop
        if robot_gt is not None:
            robot_gt.close()
    wall = time.perf_counter() - wall0
    print(f"[stack_sim] done: {world.current_time:.1f} s simulated in {wall:.1f} s wall, {steps} steps, "
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
