#!/usr/bin/env python3
"""Stream position setpoints to PX4's RAPTOR mode so the vehicle flies to a goal.

TRANSMITS to the flight controller. It never arms and never changes mode by
itself; the only mode change is the explicit `engage` command, which asks for a
confirmation first. Arm and switch into RAPTOR from the RC or QGC.

How it works: SET_POSITION_TARGET_LOCAL_NED is streamed at --rate (default
50 Hz) with position, velocity, yaw and yaw rate all set. mc_raptor rejects a
trajectory_setpoint with any of those non-finite, and treats one older than
200 ms as stale, in which case it holds the position it had at that moment.
PX4 only forwards the message to trajectory_setpoint while the current mode
accepts offboard setpoints, so nothing reaches RAPTOR before it is active.

  * Not in RAPTOR: the setpoint follows the measured position, so entering the
    mode never causes a jump.
  * On entering RAPTOR: the current position and yaw become the anchor. Goals
    are given relative to it in metres, North/East/Up (Up is positive!).
  * Goals are not sent as steps: a reference point moves toward them with
    speed and acceleration limits, with a velocity feed-forward.
  * Leaving RAPTOR clears the goal and the anchor.
  * If LOCAL_POSITION_NED stops arriving, or this script stops or crashes,
    streaming stops and RAPTOR's own 200 ms timeout makes it hold position.

    python3 raptor_goto.py                                # interactive, /dev/ttyAMA0 @ 115200
    python3 raptor_goto.py --waypoints "0,0,1; 1,0,1; 1,1,1; 0,0,1" --dwell 3
    python3 raptor_goto.py --dry-run                      # compute and print, send nothing

Interactive commands (prompt shows once RAPTOR is active):
    goto N E U [YAW_DEG]    fly to N/E/Up metres from the anchor; yaw is the NED heading
    rel dN dE dU            move relative to the current goal
    home                    back to the anchor (0 0 0)
    hold                    stop where the reference can stop
    status                  print mode, position, reference, goal
    engage yes              request the RAPTOR mode (plain `engage` only explains; does not arm)
    quit                    stop streaming (RAPTOR then holds position)

Requires pymavlink and pyserial (the ~/mav venv on the Pi has both).
"""
import argparse
import math
import queue
import sys
import threading
import time

from pymavlink import mavutil

MAV = mavutil.mavlink
PX4_MAIN_AUTO, PX4_MAIN_OFFBOARD = 4, 6
PX4_SUB_EXTERNAL1 = 11
# position + velocity + yaw + yaw rate used; acceleration ignored (NaN in trajectory_setpoint)
TYPE_MASK = (MAV.POSITION_TARGET_TYPEMASK_AX_IGNORE | MAV.POSITION_TARGET_TYPEMASK_AY_IGNORE
             | MAV.POSITION_TARGET_TYPEMASK_AZ_IGNORE)
POS_STALE_S = 0.5


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def parse_mode(s):
    """'ext1'..'ext8' or 'offboard' -> (main_mode, sub_mode or None)."""
    s = s.lower()
    if s == "offboard":
        return PX4_MAIN_OFFBOARD, None
    if s.startswith("ext") and s[3:].isdigit() and 1 <= int(s[3:]) <= 8:
        return PX4_MAIN_AUTO, PX4_SUB_EXTERNAL1 + int(s[3:]) - 1
    raise argparse.ArgumentTypeError("mode must be ext1..ext8 or offboard")


def mode_name(custom_mode):
    main, sub = (custom_mode >> 16) & 0xFF, (custom_mode >> 24) & 0xFF
    if main == PX4_MAIN_AUTO and PX4_SUB_EXTERNAL1 <= sub < PX4_SUB_EXTERNAL1 + 8:
        return f"EXTERNAL{sub - PX4_SUB_EXTERNAL1 + 1}"
    if main == PX4_MAIN_AUTO:
        return "AUTO." + {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL", 6: "LAND"}.get(sub, str(sub))
    return {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED"}.get(main, f"main{main}")


class Reference:
    """A point that moves toward a goal with speed/acceleration limits (sqrt braking profile)."""

    def __init__(self, vmax, amax, yaw_rate_max):
        self.vmax, self.amax, self.yaw_rate_max = vmax, amax, yaw_rate_max
        self.reset([0.0, 0.0, 0.0], 0.0)

    def reset(self, p, yaw):
        self.p, self.v = list(p), [0.0, 0.0, 0.0]
        self.yaw, self.yaw_rate = yaw, 0.0
        self.goal, self.goal_yaw = list(p), yaw

    def arrived(self):
        return self.p == self.goal and self.v == [0.0, 0.0, 0.0] and self.yaw == self.goal_yaw

    def stopping_point(self):
        s = math.sqrt(sum(x * x for x in self.v))
        k = s / (2 * self.amax) if s > 0 else 0.0
        return [p + v * k for p, v in zip(self.p, self.v)]

    def step(self, dt):
        d = [g - p for g, p in zip(self.goal, self.p)]
        dist = math.sqrt(sum(x * x for x in d))
        speed = math.sqrt(sum(x * x for x in self.v))
        if dist < 0.01 and speed <= self.amax * dt * 1.01:  # snap only within one accel step
            self.p, self.v = list(self.goal), [0.0, 0.0, 0.0]
        else:
            # 0.8 margin on braking so the accel-limited follower does not overshoot
            v_des = min(self.vmax, math.sqrt(2 * 0.8 * self.amax * dist), dist / dt)
            v_cmd = [x / dist * v_des for x in d] if dist > 0 else [0.0, 0.0, 0.0]
            dv = [c - v for c, v in zip(v_cmd, self.v)]
            n = math.sqrt(sum(x * x for x in dv))
            lim = self.amax * dt
            if n > lim:
                dv = [x * lim / n for x in dv]
            self.v = [v + x for v, x in zip(self.v, dv)]
            self.p = [p + v * dt for p, v in zip(self.p, self.v)]
        e = wrap(self.goal_yaw - self.yaw)
        r = max(-self.yaw_rate_max, min(self.yaw_rate_max, e / dt))
        if abs(e) < 1e-3:
            self.yaw, self.yaw_rate = self.goal_yaw, 0.0
        else:
            self.yaw, self.yaw_rate = wrap(self.yaw + r * dt), r


class Streamer:
    def __init__(self, args):
        self.args = args
        self.want_main, self.want_sub = args.mode
        self.m = mavutil.mavlink_connection(args.port, baud=args.baud, autoreconnect=False,
                                            source_system=args.sysid, source_component=MAV.MAV_COMP_ID_ONBOARD_COMPUTER)
        self.ref = Reference(args.max_speed, args.max_accel, math.radians(args.max_yaw_rate))
        self.cmds = queue.Queue()
        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        self.fc_sys = None
        self.custom_mode = None
        self.armed = False
        self.pos = self.vel = None
        self.pos_t = -1e9
        self.yaw = 0.0
        self.active = False
        self.was_stale = False
        self.anchor = None
        self.waypoints = list(args.waypoints)
        self.dwell_until = None
        self.sent = 0
        self.stale_reported = False
        self.running = True

    # --- helpers -------------------------------------------------------------
    def log(self, s):
        print(f"\r[{time.monotonic() - self.t0:7.2f}] {s}", flush=True)

    def rel_to_ned(self, n, e, u):
        a = self.anchor
        return [a[0] + n, a[1] + e, a[2] - u]

    def ned_to_rel(self, p):
        a = self.anchor
        return p[0] - a[0], p[1] - a[1], a[2] - p[2]

    def in_box(self, n, e, u):
        a = self.args
        if math.hypot(n, e) > a.box_xy:
            return f"horizontal distance {math.hypot(n, e):.2f} m > --box-xy {a.box_xy}"
        if not a.up_min <= u <= a.up_max:
            return f"up {u:.2f} m outside [--up-min {a.up_min}, --up-max {a.up_max}]"
        return None

    def set_goal(self, n, e, u, yaw_deg=None):
        err = self.in_box(n, e, u)
        if err:
            self.log(f"REJECTED goal ({n:.2f}, {e:.2f}, {u:.2f}): {err}")
            return False
        self.ref.goal = self.rel_to_ned(n, e, u)
        if yaw_deg is not None:
            self.ref.goal_yaw = wrap(math.radians(yaw_deg))
        dist = math.sqrt(sum((g - p) ** 2 for g, p in zip(self.ref.goal, self.ref.p)))
        self.log(f"goal N={n:.2f} E={e:.2f} U={u:.2f} yaw={math.degrees(self.ref.goal_yaw):.0f} deg "
                 f"({dist:.2f} m away, ~{dist / self.args.max_speed:.1f} s at cruise)")
        return True

    # --- MAVLink -------------------------------------------------------------
    def receive(self):
        while True:
            msg = self.m.recv_msg()
            if msg is None:
                return
            t = msg.get_type()
            if self.fc_sys is not None and msg.get_srcSystem() != self.fc_sys:
                continue
            if t == "HEARTBEAT" and msg.get_srcComponent() == MAV.MAV_COMP_ID_AUTOPILOT1 \
                    and msg.autopilot == MAV.MAV_AUTOPILOT_PX4:
                if self.fc_sys is None:
                    self.fc_sys = msg.get_srcSystem()
                    self.log(f"FC sys={self.fc_sys} mode={mode_name(msg.custom_mode)}")
                if msg.custom_mode != self.custom_mode and self.custom_mode is not None:
                    self.log(f"mode {mode_name(self.custom_mode)} -> {mode_name(msg.custom_mode)}")
                self.custom_mode = msg.custom_mode
                armed = bool(msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)
                if armed != self.armed:
                    self.log("ARMED" if armed else "disarmed")
                self.armed = armed
            elif t == "LOCAL_POSITION_NED":
                self.pos, self.vel = [msg.x, msg.y, msg.z], [msg.vx, msg.vy, msg.vz]
                self.pos_t = time.monotonic()
            elif t == "ATTITUDE":
                self.yaw = msg.yaw
            elif t == "STATUSTEXT":
                self.log(f"FC: {msg.text}")
            elif t == "COMMAND_ACK":
                self.log(f"COMMAND_ACK cmd={msg.command} result="
                         f"{MAV.enums['MAV_RESULT'][msg.result].name if msg.result in MAV.enums['MAV_RESULT'] else msg.result}")

    def raptor_mode(self):
        if self.custom_mode is None:
            return False
        main, sub = (self.custom_mode >> 16) & 0xFF, (self.custom_mode >> 24) & 0xFF
        return main == self.want_main and (self.want_sub is None or sub == self.want_sub)

    def send_setpoint(self):
        r = self.ref
        if not self.args.dry_run:
            self.m.mav.set_position_target_local_ned_send(
                int((time.monotonic() - self.t0) * 1000) & 0xFFFFFFFF,
                self.fc_sys, MAV.MAV_COMP_ID_AUTOPILOT1, MAV.MAV_FRAME_LOCAL_NED, TYPE_MASK,
                r.p[0], r.p[1], r.p[2], r.v[0], r.v[1], r.v[2], 0, 0, 0, r.yaw, r.yaw_rate)
        self.sent += 1

    def send_heartbeat(self):
        if not self.args.dry_run:
            self.m.mav.heartbeat_send(MAV.MAV_TYPE_ONBOARD_CONTROLLER, MAV.MAV_AUTOPILOT_INVALID, 0, 0,
                                      MAV.MAV_STATE_ACTIVE)

    def request_engage(self):
        if self.fc_sys is None:
            self.log("no FC heartbeat yet")
            return
        sub = self.want_sub or 0
        self.log(f"requesting mode main={self.want_main} sub={sub} (does not arm)")
        if not self.args.dry_run:
            self.m.mav.command_long_send(self.fc_sys, MAV.MAV_COMP_ID_AUTOPILOT1, MAV.MAV_CMD_DO_SET_MODE, 0,
                                         MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, self.want_main, sub, 0, 0, 0, 0)

    # --- command handling ----------------------------------------------------
    def handle(self, line):
        w = line.split()
        if not w:
            return
        c = w[0].lower()
        try:
            if c == "quit":
                self.running = False
            elif c == "status":
                self.print_status()
            elif c == "engage":
                if len(w) == 2 and w[1] == "yes":
                    self.request_engage()
                else:
                    self.log("this requests the RAPTOR mode on the FC; type 'engage yes' to confirm")
            elif not self.active:
                self.log(f"'{c}' ignored: RAPTOR not active (mode={self.mode_str()})")
            elif c == "goto" and len(w) in (4, 5):
                self.waypoints.clear()
                self.set_goal(*map(float, w[1:4]), float(w[4]) if len(w) == 5 else None)
            elif c == "rel" and len(w) == 4:
                self.waypoints.clear()
                n, e, u = self.ned_to_rel(self.ref.goal)
                dn, de, du = map(float, w[1:4])
                self.set_goal(n + dn, e + de, u + du)
            elif c == "home":
                self.waypoints.clear()
                self.set_goal(0.0, 0.0, 0.0)
            elif c == "hold":
                self.waypoints.clear()
                self.ref.goal = self.ref.stopping_point()
                self.log("holding at stopping point")
            else:
                self.log(f"unknown or malformed command: {line.strip()}")
        except ValueError as ex:
            self.log(f"bad number: {ex}")

    def mode_str(self):
        return mode_name(self.custom_mode) if self.custom_mode is not None else "?"

    def print_status(self):
        lines = [f"mode={self.mode_str()} armed={self.armed} raptor_active={self.active} "
                 f"sent={self.sent}{' (dry-run, nothing sent)' if self.args.dry_run else ''}"]
        age = time.monotonic() - self.pos_t
        if self.pos is None:
            lines.append("no LOCAL_POSITION_NED received")
        elif self.anchor is None:
            lines.append(f"pos NED=({self.pos[0]:.2f}, {self.pos[1]:.2f}, {self.pos[2]:.2f}) age={age:.2f}s, no anchor")
        else:
            f = lambda p: "(%.2f, %.2f, %.2f)" % self.ned_to_rel(p)
            err = math.sqrt(sum((a - b) ** 2 for a, b in zip(self.pos, self.ref.p)))
            lines.append(f"NEU from anchor: pos={f(self.pos)} ref={f(self.ref.p)} goal={f(self.ref.goal)} "
                         f"|pos-ref|={err:.2f} m yaw={math.degrees(self.yaw):.0f} deg "
                         f"waypoints_left={len(self.waypoints)}")
        for s in lines:
            self.log(s)

    # --- main loop -----------------------------------------------------------
    def run(self):
        dt = 1.0 / self.args.rate
        next_t = time.monotonic()
        next_hb = next_t
        while self.running:
            self.receive()
            while not self.cmds.empty():
                self.handle(self.cmds.get_nowait())
            now = time.monotonic()
            fresh = self.pos is not None and now - self.pos_t < POS_STALE_S

            active = self.raptor_mode()
            if active and not self.active:
                if fresh:
                    self.anchor = list(self.pos)
                    self.ref.reset(self.pos, self.yaw)
                    self.active = True
                    self.was_stale = False
                    self.dwell_until = None
                    self.log(f"RAPTOR active, anchor NED=({self.pos[0]:.2f}, {self.pos[1]:.2f}, {self.pos[2]:.2f}) "
                             f"yaw={math.degrees(self.yaw):.0f} deg. Goals are N/E/Up metres from here.")
                    if self.waypoints:
                        self.set_goal(*self.waypoints.pop(0))
            elif not active and self.active:
                self.active = False
                self.anchor = None
                if self.waypoints:
                    self.log(f"RAPTOR left, {len(self.waypoints)} waypoints abandoned")
                    self.waypoints.clear()
                else:
                    self.log("RAPTOR left, goal cleared")

            if not self.active and fresh:
                self.ref.reset(self.pos, self.yaw)  # follow the vehicle so activation is jump-free
            elif self.active and not fresh:
                self.was_stale = True  # RAPTOR holds by itself meanwhile; don't let the reference run ahead
            elif self.active:
                if self.was_stale:
                    # restart the reference from where the vehicle actually is, keep the goal
                    self.was_stale = False
                    goal, goal_yaw = self.ref.goal, self.ref.goal_yaw
                    self.ref.reset(self.pos, self.yaw)
                    self.ref.goal, self.ref.goal_yaw = goal, goal_yaw
                    self.log("position back: reference restarted from the measured position")
                self.ref.step(dt)
                if self.ref.arrived() and self.waypoints:
                    if self.dwell_until is None:
                        self.dwell_until = now + self.args.dwell
                        self.print_status()
                    elif now >= self.dwell_until:
                        self.dwell_until = None
                        self.set_goal(*self.waypoints.pop(0))

            if now >= next_hb:
                next_hb = now + 1.0
                self.send_heartbeat()
            if self.fc_sys is not None and fresh:
                self.send_setpoint()
                self.stale_reported = False
            elif not self.stale_reported and self.fc_sys is not None:
                self.stale_reported = True
                self.log("LOCAL_POSITION_NED stale or missing: NOT streaming (RAPTOR will hold position)")

            next_t += dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()
        self.log(f"stopped streaming after {self.sent} setpoints; RAPTOR holds position after 200 ms")
        self.m.close()


def parse_waypoints(s):
    wps = []
    for part in filter(None, (p.strip() for p in s.split(";"))):
        v = [float(x) for x in part.split(",")]
        if len(v) not in (3, 4):
            raise argparse.ArgumentTypeError(f"waypoint '{part}' must be N,E,U or N,E,U,YAW_DEG")
        wps.append(v)
    return wps


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument("--port", default="/dev/ttyAMA0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--sysid", type=int, default=1, help="our MAVLink system id (PX4 is also 1 by default)")
    p.add_argument("--mode", type=parse_mode, default="ext1",
                   help="how RAPTOR shows in HEARTBEAT: ext1..ext8 (MC_RAPTOR_OFFB=0, default ext1) "
                        "or offboard (MC_RAPTOR_OFFB=1)")
    p.add_argument("--rate", type=float, default=50.0, help="setpoint rate, Hz (RAPTOR times out at 200 ms)")
    p.add_argument("--max-speed", type=float, default=0.5, help="m/s")
    p.add_argument("--max-accel", type=float, default=0.5, help="m/s^2")
    p.add_argument("--max-yaw-rate", type=float, default=30.0, help="deg/s")
    p.add_argument("--box-xy", type=float, default=2.0, help="max horizontal distance from anchor, m")
    p.add_argument("--up-min", type=float, default=-0.5, help="min height relative to anchor, m")
    p.add_argument("--up-max", type=float, default=2.0, help="max height relative to anchor, m")
    p.add_argument("--waypoints", type=parse_waypoints, default=[], help='"N,E,U[,YAW]; ..." run once on activation')
    p.add_argument("--dwell", type=float, default=3.0, help="seconds to wait at each waypoint")
    p.add_argument("--dry-run", action="store_true", help="receive and compute, never transmit")
    args = p.parse_args()
    if args.rate < 10:
        p.error("--rate below 10 Hz would trip RAPTOR's 200 ms setpoint timeout")

    s = Streamer(args)
    for wp in args.waypoints:
        err = s.in_box(*wp[:3])
        if err:
            p.error(f"waypoint {wp}: {err}")
    s.log(f"{args.port} @ {args.baud}, {args.rate:.0f} Hz, vmax={args.max_speed} m/s, "
          f"box xy<={args.box_xy} m up=[{args.up_min},{args.up_max}] m"
          f"{', DRY RUN' if args.dry_run else ''}. Waiting for RAPTOR mode; this script does not arm.")

    def reader():
        for line in sys.stdin:
            s.cmds.put(line)
        # stdin closed: keep streaming (waypoints / hold) until Ctrl-C

    threading.Thread(target=reader, daemon=True).start()
    try:
        s.run()
    except KeyboardInterrupt:
        s.log(f"Ctrl-C: stopped streaming after {s.sent} setpoints; RAPTOR holds position after 200 ms")


if __name__ == "__main__":
    main()
