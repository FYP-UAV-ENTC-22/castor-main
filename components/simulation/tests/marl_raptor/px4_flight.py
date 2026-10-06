#!/usr/bin/env python3
"""The DDS side of the PX4 run: commands three PX4s, tensions and lifts the payload, then flies the MARL policy.

Runs in a CASTOR vehicle image (ROS 2, px4_msgs), next to the three vehicle containers of px4_stack.yml. Everything
it knows about a drone, and everything it tells one, goes through PX4's uXRCE-DDS link:

  reads   <ns>/vehicle/odom, <ns>/vehicle/state       (vehicle_interface: PX4 vehicle_odometry and vehicle_status)
          /px4_<i>/fmu/out/vehicle_local_position     the geodetic origin of that PX4's local frame
  writes  /px4_<i>/fmu/in/vehicle_command             arm, take off, change mode
          <ns>/vehicle/setpoint                       position setpoints, which vehicle_interface forwards to PX4's
                                                      trajectory_setpoint for RAPTOR

The payload pose comes from the simulator (px4_rig.py, UDP) and is republished here as /team/payload/odom.

Frames. Each PX4 reports positions in its own local frame, whose origin is wherever its estimator started. PX4
also reports where that origin is on the globe; with the datum (the world origin's coordinates) that gives each
drone's offset, and everything is put into one east-north-up world frame before the policy sees it. Setpoints go
back the other way.

Sequence, each step on the operator's word (or by itself with --auto):

  takeoff   PX4 arms and takes off in its own take-off mode, to a height where the cables are still slack
  raptor    PX4 switches to the RAPTOR mode and holds position
  tension   setpoints rise slowly: the cables go taut and the payload lifts
  rl        after a check that every cable is taut and the payload airborne, the policy moves the setpoints
  goal X Y Z [ROLL PITCH YAW]      world frame, metres and degrees
  stop      setpoints freeze: RAPTOR holds
  land      PX4 land mode
  status

    docker compose -f px4_stack.yml run --rm flight                     # interactive
    docker compose -f px4_stack.yml run --rm flight python3 px4_flight.py --auto --goal=0.6,0.4,1.2,0,0,30
"""

import argparse
import json
import math
import os
import socket
import sys
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from castor_interfaces.msg import PositionSetpoint, VehicleState

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import marl_policy as MP  # noqa: E402

NAV_AUTO_LOITER, NAV_AUTO_TAKEOFF, NAV_AUTO_LAND, NAV_RAPTOR = 4, 17, 18, 23  # RAPTOR registers as EXTERNAL1
TAUT_TOLERANCE = 0.05  # m, on spans measured from GNSS positions
GOAL_BOX = (np.array([-1.0, -1.0, 0.5]), np.array([1.0, 1.0, 1.5]))  # the goal positions seen in training

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--auto", action="store_true", help="run the whole sequence without waiting for the operator")
parser.add_argument("--goal", action="append", default=[], metavar="X,Y,Z[,ROLL,PITCH,YAW]",
                    help="with --auto: goals flown in order, --goal_time seconds each")
parser.add_argument("--goal_time", type=float, default=12.0)
parser.add_argument("--lift_height", type=float, default=1.0, help="payload height above the ground after the lift")
parser.add_argument("--lift_speed", type=float, default=0.15, help="m/s while the cables go taut and the payload lifts")
parser.add_argument("--trained_on", choices=("falcon", "s500"), default="falcon",
                    help="the task whose policy is flown (models/<trained_on>/): sets the point given to the policy as the "
                         "drone's position, the step scale, the setpoint speed cap and the velocity filter. Give "
                         "px4_rig.py the same, for the rig. See the README")
parser.add_argument("--step_scale", type=float, default=None,
                    help="metres a unit policy action moves a setpoint per 20 ms step. Default: 0.015 for a Falcon "
                         "policy (trained with 0.05, see the README), training's 0.02 for an S500 policy")
parser.add_argument("--max_speed", type=float, default=None,
                    help="m/s, cap on how fast a policy setpoint moves. Default: none for a Falcon policy, training's "
                         "1.0 for an S500 policy")
parser.add_argument("--velocity_gain", type=float, default=1.0,
                    help="scale on the velocity feedforward sent with each policy setpoint (training: 1); see the README")
parser.add_argument("--payload_filter", type=float, default=0.0, metavar="SECONDS",
                    help="time constant of a low-pass filter on the payload pose the policy is given; 0 = none")
parser.add_argument("--velocity_filter", type=float, default=None, metavar="SECONDS",
                    help="time constant of a low-pass filter on the velocity feedforward; 0 = none. Default: 0.1 for "
                         "a Falcon policy, 0 for an S500 policy, which was trained with the feedforward as it is")
parser.add_argument("--models", default=None, help="model package directory; default: the --trained_on task's")
parser.add_argument("--payload_port", type=int, default=14600)
parser.add_argument("--goal_port", type=int, default=14601)
parser.add_argument("--num_drones", type=int, default=3)
args = parser.parse_args()

trained = MP.TRAINED_ON[args.trained_on]
if args.models is None:
    args.models = MP.models_dir(args.trained_on)
if args.step_scale is None:
    args.step_scale = 0.015 if args.trained_on == "falcon" else trained["step_scale"]
if args.max_speed is None:
    args.max_speed = trained["max_speed"]
if args.velocity_filter is None:
    args.velocity_filter = 0.1 if args.trained_on == "falcon" else 0.0


def quat_to_matrix(w, x, y, z):
    return MP.quat_wxyz_to_matrix((w, x, y, z))


def versioned(topic, msg_type):
    version = getattr(msg_type, "MESSAGE_VERSION", 0)
    return topic if version == 0 else f"{topic}_v{version}"


class Drone:
    def __init__(self, index):
        self.index = index
        self.ns = f"drone{index + 1}"
        self.px4_ns = f"px4_{index + 1}"
        self.position = None  # local ENU, body origin
        self.rotation = None  # body FLU -> ENU
        self.lin_vel = self.ang_vel = None  # ENU
        self.armed = False
        self.nav_state = -1
        self.connected = False
        self.ref = None  # (lat, lon, alt) of the local frame's origin
        self.offset = None  # that origin in the world frame
        self.setpoint = None  # world, body origin: what is streamed to RAPTOR
        self.setpoint_vel = np.zeros(3)

    @property
    def ready(self):
        return self.position is not None and self.offset is not None and self.connected

    @property
    def world(self):
        return self.position + self.offset


class Flight(Node):
    def __init__(self):
        super().__init__("px4_flight")
        self.n = args.num_drones
        self.drones = [Drone(i) for i in range(self.n)]
        self.phase = "wait"
        self.requests = []  # operator commands, filled by the input thread
        self.lock = threading.Lock()
        self.payload = None  # the latest simulator message
        self.rig = None
        self.datum = None
        self.t = 0.0
        self.phase_t0 = 0.0
        self.targets = None
        self.goal_pos = self.goal_rot = None
        self.auto_goals = [self.parse_goal(g.split(",")) for g in args.goal]
        self.auto_goal_index = -1
        self.report = []
        self.motion = []
        self.observations = [MP.Observation(i, self.n) for i in range(self.n)]
        self.integrators = [MP.SetpointIntegrator(step_scale=args.step_scale, max_speed=args.max_speed)
                            for _ in range(self.n)]
        self.policies = [MP.OnnxPolicy(os.path.join(args.models, f"policy_falcon{i + 1}.onnx")) for i in range(self.n)]

        self.command_pubs, self.setpoint_pubs = [], []
        for d in self.drones:
            self.create_subscription(Odometry, f"/{d.ns}/vehicle/odom", lambda m, d=d: self.on_odom(d, m), 10)
            self.create_subscription(VehicleState, f"/{d.ns}/vehicle/state", lambda m, d=d: self.on_state(d, m), 10)
            self.create_subscription(
                VehicleLocalPosition, versioned(f"/{d.px4_ns}/fmu/out/vehicle_local_position", VehicleLocalPosition),
                lambda m, d=d: self.on_local_position(d, m), qos_profile_sensor_data)
            self.command_pubs.append(self.create_publisher(VehicleCommand, f"/{d.px4_ns}/fmu/in/vehicle_command", 10))
            self.setpoint_pubs.append(self.create_publisher(PositionSetpoint, f"/{d.ns}/vehicle/setpoint", 10))
        self.payload_pub = self.create_publisher(Odometry, "/team/payload/odom", 10)

        self.payload_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.payload_in.bind(("127.0.0.1", args.payload_port))
        self.payload_in.setblocking(False)
        self.goal_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # The simulator sends the payload pose every 20 ms of simulated time; each one is a control step, so the
        # policy runs at 50 Hz of the time PX4 lives in, however fast the simulation runs.
        self.create_timer(0.002, self.poll_payload)
        self.say(f"policy trained on the {args.trained_on} task: step scale {args.step_scale} m, setpoint speed cap "
                 f"{args.max_speed} m/s, velocity filter {args.velocity_filter} s")
        self.say("waiting for three PX4s (odometry, local-frame origin) and the payload pose")

    def say(self, text):
        print(f"[flight] t = {self.t:7.2f} s  {text}", flush=True)

    # ------------------------------------------------------------------------------------------------ PX4 -> here
    def on_odom(self, d, m):
        p, q, v, w = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear, m.twist.twist.angular
        d.position = np.array([p.x, p.y, p.z])
        d.rotation = quat_to_matrix(q.w, q.x, q.y, q.z)
        # nav_msgs/Odometry carries the twist in the body frame
        d.lin_vel = d.rotation @ np.array([v.x, v.y, v.z])
        d.ang_vel = d.rotation @ np.array([w.x, w.y, w.z])

    def on_state(self, d, m):
        d.armed, d.nav_state, d.connected = m.armed, m.nav_state, m.fc_connected

    def on_local_position(self, d, m):
        if m.xy_global and m.z_global:
            d.ref = (m.ref_lat, m.ref_lon, m.ref_alt)
            if self.datum is not None:
                d.offset = self.geodetic_to_world(*d.ref)

    def geodetic_to_world(self, lat, lon, alt):
        """East-north-up position of a geodetic point relative to the datum. Small-area approximation."""
        lat0, lon0, alt0 = self.datum
        radius = self.rig["earth_radius"]
        north = math.radians(lat - lat0) * radius
        east = math.radians(lon - lon0) * radius * math.cos(math.radians(lat0))
        return np.array([east, north, alt - alt0])

    # ------------------------------------------------------------------------------------------------ here -> PX4
    def command(self, d, command, param1=float("nan"), param2=float("nan")):
        m = VehicleCommand()
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        m.command = command
        m.param1, m.param2 = float(param1), float(param2)
        m.param3 = m.param4 = m.param7 = float("nan")
        m.param5 = m.param6 = float("nan")
        m.target_system = d.index + 1  # MAV_SYS_ID of that PX4 instance
        m.target_component = 1
        m.source_system = 255
        m.source_component = 1
        m.from_external = True
        self.command_pubs[d.index].publish(m)

    def set_mode(self, d, nav_state):
        self.command(d, VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, nav_state)

    def publish_setpoint(self, d):
        local = d.setpoint - d.offset
        m = PositionSetpoint()
        m.header.stamp = self.get_clock().now().to_msg()
        m.position = Point(x=float(local[0]), y=float(local[1]), z=float(local[2]))
        m.velocity = Vector3(x=float(d.setpoint_vel[0]), y=float(d.setpoint_vel[1]), z=float(d.setpoint_vel[2]))
        m.yaw = float(self.rig["yaws"][d.index])  # ENU: the formation heading, each drone facing outward
        m.yaw_rate = 0.0
        self.setpoint_pubs[d.index].publish(m)

    # ------------------------------------------------------------------------------------------------ payload
    def poll_payload(self):
        message = None
        try:
            while True:
                message = json.loads(self.payload_in.recv(65536).decode())
        except BlockingIOError:
            pass
        if message is None:
            return
        self.payload, self.rig, self.datum, self.t = message, message["rig"], message["datum"], message["t"]
        w, x, y, z = message["orientation_wxyz"]
        self.payload_pos, self.payload_rot = self.filter_payload(np.array(message["position"]),
                                                                 quat_to_matrix(w, x, y, z))
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "world"
        odom.child_frame_id = "payload"
        odom.pose.pose.position = Point(**dict(zip("xyz", map(float, self.payload_pos))))
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x = float(w), float(x)
        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z = float(y), float(z)
        self.payload_pub.publish(odom)
        for d in self.drones:
            if d.ref is not None and d.offset is None:
                d.offset = self.geodetic_to_world(*d.ref)
        self.step()

    def filter_payload(self, position, rotation):
        """First-order low-pass on the payload pose, the job a payload state estimator does on the aircraft."""
        if args.payload_filter <= 0.0 or self.payload is None or not hasattr(self, "payload_pos"):
            return position, rotation
        a = MP.POLICY_DT / (args.payload_filter + MP.POLICY_DT)
        position = self.payload_pos + a * (position - self.payload_pos)
        # move the filtered rotation a fraction of the way to the measured one, along the shortest rotation
        delta = self.payload_rot.T @ rotation
        angle = math.acos(max(-1.0, min(1.0, (np.trace(delta) - 1.0) / 2.0)))
        if angle > 1e-9:
            axis = np.array([delta[2, 1] - delta[1, 2], delta[0, 2] - delta[2, 0], delta[1, 0] - delta[0, 1]])
            axis = axis / (2.0 * math.sin(angle))
            K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
            step = a * angle
            rotation = self.payload_rot @ (np.eye(3) + math.sin(step) * K + (1 - math.cos(step)) * K @ K)
        else:
            rotation = self.payload_rot
        return position, rotation

    # ------------------------------------------------------------------------------------------------ sequence
    def enter(self, phase):
        self.motion_summary()
        self.phase, self.phase_t0 = phase, self.t
        self.say(phase)

    def motion_summary(self):
        """How steadily the drones flew in the phase that is ending, from PX4's own attitude and rates."""
        if len(self.motion) > 50 and any(d.armed for d in self.drones):
            m = np.array(self.motion[len(self.motion) // 3:])  # skip the phase's start transient
            self.say(f"   drones in '{self.phase.split(':')[0][:28]}': tilt mean {m[:, 0].mean():.1f} max {m[:, 0].max():.1f} deg, "
                     f"roll/pitch rate rms {np.sqrt((m[:, 1] ** 2).mean()):.0f} max {m[:, 1].max():.0f} deg/s")
        self.motion = []

    def next_request(self):
        with self.lock:
            return self.requests.pop(0) if self.requests else None

    def spans(self):
        """Anchor-to-mount distance of each cable, from the positions PX4 and the payload report."""
        mount, anchors = np.array(self.rig["mount_local"]), np.array(self.rig["anchors_local"])
        return np.array([np.linalg.norm((d.world + d.rotation @ mount) - (self.payload_pos + self.payload_rot @ a))
                         for d, a in zip(self.drones, anchors)])

    def step(self):
        """One 20 ms control step."""
        if all(d.rotation is not None for d in self.drones):
            tilt = max(math.degrees(math.acos(max(-1.0, min(1.0, d.rotation[2, 2])))) for d in self.drones)
            rate = max(math.degrees(float(np.linalg.norm((d.rotation.T @ d.ang_vel)[:2]))) for d in self.drones)
            self.motion.append((tilt, rate))
        request = self.next_request()
        elapsed = self.t - self.phase_t0
        if request:
            # which word each phase is waiting for; anything else is refused out loud, never silently dropped
            expected = {"ready": "takeoff", "hovering": "raptor", "in RAPTOR": "tension", "hold": "rl", "rl": "goal"}
            wanted = next((w for prefix, w in expected.items() if self.phase.startswith(prefix)), None)
            if request[0] not in ("status", "land", "stop", wanted):
                self.say(f"'{' '.join(request)}' refused in phase '{self.phase}'"
                         + (f"; expected: {wanted}" if wanted else ""))
        if request and request[0] == "status":
            self.print_status()
        if request and request[0] == "land":
            for d in self.drones:
                self.set_mode(d, NAV_AUTO_LAND)
            return self.enter("land: PX4 land mode")
        if request and request[0] == "stop" and self.phase in ("tension", "hold", "rl"):
            for d in self.drones:
                d.setpoint_vel = np.zeros(3)
            return self.enter("hold")

        if self.phase == "wait":
            if all(d.ready for d in self.drones) and elapsed > 2.0:
                for d in self.drones:
                    self.say(f"{d.ns}: local-frame origin at {np.round(d.offset, 3)} m in the world frame, "
                             f"drone at {np.round(d.world, 3)}")
                if self.rig["trained_on"] != args.trained_on:
                    self.say(f"WARNING px4_rig.py was started with --trained_on {self.rig['trained_on']}, this with "
                             f"--trained_on {args.trained_on}: the rig is not the one this policy was trained on")
                self.enter("ready: on the ground, disarmed. Next: takeoff")
        elif self.phase.startswith("ready"):
            if (request and request[0] == "takeoff") or (args.auto and elapsed > 1.0):
                self.enter("takeoff")
        elif self.phase == "takeoff":
            # PX4's own take-off mode and controllers. Repeated until each PX4 reports it, as commands can be lost.
            if int(elapsed / 0.02) % 25 == 0:
                for d in self.drones:
                    if d.nav_state not in (NAV_AUTO_TAKEOFF, NAV_AUTO_LOITER) or not d.armed:
                        self.set_mode(d, NAV_AUTO_TAKEOFF)
                        self.command(d, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if all(d.armed and d.nav_state == NAV_AUTO_LOITER for d in self.drones) and elapsed > 3.0:
                heights = [round(float(d.world[2]), 2) for d in self.drones]
                self.enter(f"hovering in PX4 hold at {heights} m, cables slack. Next: raptor")
            elif elapsed > 40.0:
                self.enter("FAILED: take-off did not complete; see status")
        elif self.phase.startswith("hovering"):
            if (request and request[0] == "raptor") or (args.auto and elapsed > 3.0):
                self.enter("raptor")
        elif self.phase == "raptor":
            if int(elapsed / 0.02) % 25 == 0:
                for d in self.drones:
                    if d.nav_state != NAV_RAPTOR:
                        self.set_mode(d, NAV_RAPTOR)
            if all(d.nav_state == NAV_RAPTOR for d in self.drones):
                self.enter("in RAPTOR, holding position without setpoints. Next: tension")
            elif elapsed > 10.0:
                self.enter("FAILED: PX4 did not enter the RAPTOR mode; see status")
        elif self.phase.startswith("in RAPTOR"):
            if (request and request[0] == "tension") or (args.auto and elapsed > 3.0):
                self.start_tension()
        elif self.phase == "tension":
            self.tension_step(elapsed)
        elif self.phase == "hold":
            for d in self.drones:
                self.publish_setpoint(d)
            if (request and request[0] == "rl") or (args.auto and elapsed > 4.0 and not self.report):
                self.start_rl()
        elif self.phase == "rl":
            if request and request[0] == "goal":
                self.set_goal(*self.parse_goal(request[1:]))
            self.rl_step(elapsed)
        elif self.phase.startswith("FAILED") and args.auto:
            self.finish(False)

    def start_tension(self):
        """From where each drone hovers to its place in the lifted formation, around where the payload lies."""
        yaw = math.atan2(self.payload_rot[1, 0], self.payload_rot[0, 0])
        rot = MP.euler_xyz_to_matrix(0.0, 0.0, yaw)
        lift = args.lift_height - self.payload_pos[2]
        self.starts = [d.world.copy() for d in self.drones]
        self.targets = [self.payload_pos + rot @ np.array(slot) + np.array([0.0, 0.0, lift])
                        for slot in self.rig["slots"]]
        climb = max(float(np.linalg.norm(b - a)) for a, b in zip(self.starts, self.targets))
        self.tension_time = climb / args.lift_speed
        for d, start in zip(self.drones, self.starts):
            d.setpoint = start.copy()
        self.say(f"cable spans {np.round(self.spans(), 2)} m of {self.rig['cable_length']} m; climbing "
                 f"{climb:.2f} m in {self.tension_time:.0f} s")
        self.enter("tension")

    def tension_step(self, elapsed):
        s = min(elapsed / self.tension_time, 1.0)
        shape = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        rate = (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / self.tension_time
        for d, start, target in zip(self.drones, self.starts, self.targets):
            d.setpoint = start + (target - start) * shape
            d.setpoint_vel = (target - start) * rate
            self.publish_setpoint(d)
        if s >= 1.0:
            for d in self.drones:
                d.setpoint_vel = np.zeros(3)
            self.enter("hold")
            self.say(f"cable spans {np.round(self.spans(), 3)} m, payload at {self.payload_pos[2]:.2f} m. Next: rl")

    def start_rl(self):
        spans = self.spans()
        taut = spans >= self.rig["cable_length"] - TAUT_TOLERANCE
        bottom = self.payload_pos[2] - self.rig["payload_height"] / 2
        ok = bool(taut.all() and bottom > 0.2 and all(d.nav_state == NAV_RAPTOR for d in self.drones))
        self.say(f"hand-over check: cable spans {np.round(spans, 3)} m ({'all taut' if taut.all() else 'NOT all taut'}), "
                 f"payload {bottom:.2f} m above the ground, modes {[d.nav_state for d in self.drones]}")
        if not ok:
            self.say("hand-over refused: RAPTOR keeps holding the formation")
            self.report.append(("hand-over", False))
            if args.auto:
                self.finish(False)
            return
        # the point the policy was trained to call the drone's position, see marl_policy.TRAINED_ON
        self.point_local = MP.policy_point(args.trained_on, self.rig["mount_local"], self.rig["com_local"])
        for i, d in enumerate(self.drones):
            self.integrators[i].seed(d.world + d.rotation @ self.point_local)
            self.observations[i].reset()
        yaw = math.atan2(self.payload_rot[1, 0], self.payload_rot[0, 0])
        self.goal_pos, self.goal_rot = self.payload_pos.copy(), MP.euler_xyz_to_matrix(0.0, 0.0, yaw)
        self.errors = []
        self.enter("rl")
        self.say("the policy holds the payload where it is. Next: goal X Y Z [ROLL PITCH YAW]")

    def parse_goal(self, words):
        v = [float(x) for x in words]
        v = v + [0.0] * (6 - len(v))
        return np.array(v[:3]), MP.euler_xyz_to_matrix(*np.radians(v[3:6]))

    def set_goal(self, position, rotation):
        inside = np.clip(position, GOAL_BOX[0], GOAL_BOX[1])
        if not np.allclose(inside, position):
            self.say(f"goal {position} is outside the trained box; using {inside}")
        self.goal_pos, self.goal_rot = inside, rotation
        self.errors = []
        self.say(f"goal: position {np.round(inside, 2)} m ({np.linalg.norm(inside - self.payload_pos):.2f} m away)")

    def rl_step(self, elapsed):
        if args.auto:
            index = int((elapsed - 3.0) // args.goal_time) if elapsed >= 3.0 else -1
            if index != self.auto_goal_index:
                self.score_goal()
                if index >= len(self.auto_goals):
                    return self.finish(all(ok for _, ok in self.report))
                self.auto_goal_index = index
                self.set_goal(*self.auto_goals[index])
        setpoints = []
        for i, d in enumerate(self.drones):
            arm = d.rotation @ self.point_local
            point = d.world + arm
            obs = self.observations[i].push(
                payload_pos=self.payload_pos, payload_rot=self.payload_rot, drone_pos=point, drone_rot=d.rotation,
                drone_lin_vel=d.lin_vel + np.cross(d.ang_vel, arm), drone_ang_vel=d.ang_vel, goal_pos=self.goal_pos,
                goal_rot=self.goal_rot)
            position, velocity = self.integrators[i].step(self.policies[i](obs), point)
            # RAPTOR flies the body origin; the policy's setpoint is for its own point
            velocity = velocity * args.velocity_gain
            if args.velocity_filter > 0.0:
                a = MP.POLICY_DT / (args.velocity_filter + MP.POLICY_DT)
                velocity = d.setpoint_vel + a * (velocity - d.setpoint_vel)
            d.setpoint, d.setpoint_vel = position - self.point_local, velocity
            self.publish_setpoint(d)
            setpoints.append(position.tolist())
        rel = self.goal_rot @ self.payload_rot.T
        self.errors.append((float(np.linalg.norm(self.goal_pos - self.payload_pos)),
                            math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(rel) - 1) / 2))))))
        q = self.matrix_to_wxyz(self.goal_rot)
        self.goal_out.sendto(json.dumps({"position": self.goal_pos.tolist(), "orientation_wxyz": q,
                                         "setpoints": setpoints}).encode(), ("127.0.0.1", args.goal_port))

    @staticmethod
    def matrix_to_wxyz(R):
        w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
        if w < 1e-6:
            return [0.0, 1.0, 0.0, 0.0]
        return [w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)]

    def score_goal(self):
        """Mean error over the last two seconds on the goal that just ended."""
        if not self.errors:
            return
        tail = np.array(self.errors[-100:])
        name = "holding at hand-over" if self.auto_goal_index < 0 else f"goal {self.auto_goal_index}"
        ok = bool(tail[:, 0].mean() < 0.15)
        self.report.append((name, ok))
        truth = self.payload["truth"]
        # when the payload got within 5 cm and 3 degrees of the goal and stayed there
        e = np.array(self.errors)
        outside = np.flatnonzero((e[:, 0] > 0.05) | (e[:, 1] > 3.0))
        settled = "at once" if outside.size == 0 else (
            "not yet" if outside[-1] == len(e) - 1 else f"after {(outside[-1] + 1) * MP.POLICY_DT:.1f} s")
        self.say(f"{name}: last 2 s mean {tail[:, 0].mean():.3f} m / {tail[:, 1].mean():.1f} deg, settled {settled}  "
                 f"{'ok' if ok else 'MISSED'}  (true cable spans {truth['cable_span']} m)")
        self.motion_summary()

    def finish(self, ok):
        self.say("PASS" if ok else "FAIL")
        self.say("the policy has stopped; RAPTOR holds the last setpoints. `land` or stop the simulator to end.")
        self.result = ok
        raise SystemExit(0 if ok else 1)

    def print_status(self):
        for d in self.drones:
            where = np.round(d.world, 2) if d.ready else None
            self.say(f"{d.ns}: connected {d.connected}, armed {d.armed}, nav_state {d.nav_state}, world position {where}")
        if self.payload is not None:
            self.say(f"payload at {np.round(self.payload_pos, 2)}; phase: {self.phase}")
            if all(d.ready for d in self.drones):
                self.say(f"cable spans from GNSS {np.round(self.spans(), 3)} m of {self.rig['cable_length']} m")


def read_operator(node):
    for line in sys.stdin:
        words = line.replace(",", " ").split()
        if words:
            with node.lock:
                node.requests.append(words)


def main():
    rclpy.init()
    node = Flight()
    if not args.auto:
        print(__doc__.split("Sequence,")[1].split("docker compose")[0])
        threading.Thread(target=read_operator, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as e:
        sys.exit(e.code)


if __name__ == "__main__":
    main()
