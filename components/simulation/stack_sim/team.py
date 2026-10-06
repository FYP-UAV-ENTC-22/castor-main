"""The ground station's side of stack_sim's mission commands, run by stack_sim.sh in one container on domain 20 (the
system image: rclpy and castor_interfaces). Every drone's mission state reaches domain 20 over its zenoh bridge.

    team.py takeoff DRONE...            wait until every drone is IDLE (printing who is not), send take-off, then follow
                                        each drone until the team is in RAPTOR (HOVER or later) or something fails
    team.py land DRONE...               send land, follow until every drone is on the ground
    team.py goal X Y Z YAW_DEG DRONE... send a payload goal once the team is READY (or flying one)
    team.py mission DRONE...            every drone's mission state, once
    team.py watch DRONE...              every mission state change until Ctrl-C
    team.py planning DRONE...           every drone's planning status, once

A command is only sent once every drone can take it: a drone's mission node refuses a take-off that arrives before its
vehicle is connected, so sending early would leave the team split.
"""

import argparse
import math
import sys
import time

import rclpy
from castor_interfaces.msg import MissionState, PolicyStatus, TeamCommand
from geometry_msgs.msg import PoseStamped

GROUND = {"WAIT_VEHICLE", "IDLE"}
IN_RAPTOR = {"HOVER", "LIFT", "LIFTED", "READY", "MARL", "HOLD"}
PAYLOAD_READY = {"READY", "MARL", "HOLD"}
FAILED = {"FC_OVERRIDE", "LANDING"}


class Team:
    def __init__(self, drones):
        rclpy.init()
        self.node = rclpy.create_node("stack_sim_team")
        self.drones = drones
        self.state = {d: None for d in drones}
        self.status = {}
        for d in drones:
            self.node.create_subscription(MissionState, f"/{d}/system/mission",
                                          lambda m, d=d: self.state.__setitem__(d, m.state), 10)
        self.t0 = time.monotonic()

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=max(0.0, min(0.1, end - time.monotonic())))

    def say(self, text):
        print(f"[{time.monotonic() - self.t0:6.1f} s] {text}", flush=True)

    def states(self):
        return ", ".join(f"{d} {self.state[d] or '(no state yet)'}" for d in self.drones)

    def wait_all(self, accept, what, timeout, every=2.0):
        """Spin until accept(state) holds for every drone; print who is not there every `every` s."""
        end, last = time.monotonic() + timeout, 0.0
        while True:
            self.spin(0.2)
            if all(s is not None and accept(s) for s in self.state.values()):
                return True
            now = time.monotonic()
            if now > end:
                self.say(f"gave up after {timeout:.0f} s waiting for {what}: {self.states()}")
                return False
            if now - last >= every:
                self.say(f"waiting for {what}: {self.states()}")
                last = now

    def follow(self, done, fail, what, timeout):
        """Print every state change until done(states) or fail(state) for some drone, or the timeout."""
        end, seen = time.monotonic() + timeout, dict(self.state)
        while time.monotonic() < end:
            self.spin(0.2)
            for d in self.drones:
                if self.state[d] != seen[d]:
                    self.say(f"{d}: {seen[d]} -> {self.state[d]}")
                    seen[d] = self.state[d]
            if done(self.state):
                return True
            bad = [d for d in self.drones if self.state[d] is not None and fail(self.state[d])]
            if bad:
                self.say(f"stopped: {', '.join(f'{d} {self.state[d]}' for d in bad)} (see that drone's system log)")
                return False
        self.say(f"no {what} after {timeout:.0f} s: {self.states()}")
        return False

    def publish(self, msg_type, topic, msg, times=3):
        pub = self.node.create_publisher(msg_type, topic, 10)
        # Wait until the ground station's bridge has matched, so the samples have a route to the drones.
        end = time.monotonic() + 5.0
        while pub.get_subscription_count() == 0 and time.monotonic() < end:
            self.spin(0.1)
        for _ in range(times):
            pub.publish(msg)
            self.spin(0.1)

    def command(self, name):
        m = TeamCommand()
        m.stamp = self.node.get_clock().now().to_msg()
        m.command = name
        self.publish(TeamCommand, "/team/command", m)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["takeoff", "land", "goal", "mission", "watch", "planning"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--timeout", type=float, default=180.0, help="s to wait for the team to be ready")
    a = ap.parse_args()
    args = a.args
    goal = None
    if a.cmd == "goal":
        if len(args) < 5:
            ap.error("goal X Y Z YAW_DEG DRONE...")
        goal, args = [float(x) for x in args[:4]], args[4:]
    if not args:
        ap.error("no drones (stack_sim.sh passes them)")
    team = Team(args)

    if a.cmd in ("mission", "planning"):
        if a.cmd == "planning":
            for d in team.drones:
                team.node.create_subscription(PolicyStatus, f"/{d}/planning/status",
                                              lambda m, d=d: team.status.__setitem__(d, m), 10)
        team.spin(2.5)  # mission state is 5 Hz, planning status 1 Hz
        for d in team.drones:
            if a.cmd == "mission":
                print(f"{d}: {team.state[d] or '(no message)'}")
                continue
            s = team.status.get(d)
            if s is None:
                print(f"{d}: (no message)")
                continue
            print(f"{d}: model {s.model_id or '-'} loaded={s.model_loaded} mode={s.mode} active={s.active}"
                  f"{' (' + s.inactive_reason + ')' if s.inactive_reason else ''} lift_done={s.lift_done}"
                  f" handover_ok={s.handover_ok}{' (' + s.handover_reason + ')' if s.handover_reason else ''}"
                  f" cable_span={s.cable_span:.3f} payload_z={s.payload_z:.3f} goal_err={s.goal_position_error:.3f} m / {math.degrees(s.goal_orientation_error):.1f} deg"
                  f" steps/s={s.loop_rate_hz:.1f}")
        return 0

    if a.cmd == "watch":
        team.say(team.states())
        try:
            team.follow(lambda s: False, lambda s: False, "end", math.inf)
        except KeyboardInterrupt:
            pass
        return 0

    if a.cmd == "takeoff":
        if not team.wait_all(lambda s: s == "IDLE", "every drone IDLE", a.timeout):
            return 1
        team.say("all IDLE: sending take-off")
        team.command(TeamCommand.TAKEOFF)
        # Each drone leaves IDLE within a step of the command (its log says why if it refuses)...
        if not team.follow(lambda s: all(x not in GROUND for x in s.values()), lambda s: s in FAILED,
                           "drone leaving IDLE", 10.0):
            return 1
        # ...then PX4's take-off and the hand-over to RAPTOR; back on the ground means arming or take-off failed.
        ok = team.follow(lambda s: all(x in IN_RAPTOR for x in s.values()),
                         lambda s: s in FAILED or s in GROUND, "team in RAPTOR", 240.0)
        if ok:
            team.say("team in RAPTOR; the mission lifts the payload by itself (stack_sim.sh watch), then waits for "
                     "a goal in READY")
        return 0 if ok else 1

    if a.cmd == "land":
        team.spin(1.0)
        team.say(f"sending land: {team.states()}")
        team.command(TeamCommand.LAND)
        ok = team.follow(lambda s: all(x in GROUND for x in s.values() if x is not None),
                         lambda s: s == "FC_OVERRIDE", "team on the ground", 240.0)
        return 0 if ok else 1

    if a.cmd == "goal":
        if not team.wait_all(lambda s: s in PAYLOAD_READY, "every drone READY (payload lifted)", a.timeout):
            return 1
        x, y, z, yaw = goal
        m = PoseStamped()
        m.header.stamp = team.node.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.z, m.pose.orientation.w = math.sin(math.radians(yaw) / 2), math.cos(math.radians(yaw) / 2)
        team.say(f"sending goal ({x}, {y}, {z}), yaw {yaw} deg")
        team.publish(PoseStamped, "/team/goal", m)
        team.follow(lambda s: all(v in ("MARL", "HOLD") for v in s.values()), lambda s: s in FAILED, "MARL", 15.0)
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(main())
