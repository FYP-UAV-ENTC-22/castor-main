#!/usr/bin/env python3
"""Receive-only MAVLink sniffer for checking the FC <-> companion serial link.

Never transmits: it only calls recv_msg() and reads the UART's TIOCGICOUNT
counters, and reports the tx counter delta so that can be verified. Does not
send heartbeats, stream requests, commands or parameter reads.

    python3 mavlink_sniff.py                         # /dev/ttyAMA0 @ 115200, 10 s
    python3 mavlink_sniff.py --baud 921600 -t 20
    python3 mavlink_sniff.py --scan                  # try common baud rates, 5 s each
    python3 mavlink_sniff.py --live                  # print every message as it arrives
    python3 mavlink_sniff.py --live --types HEARTBEAT,ATTITUDE
    python3 mavlink_sniff.py --live --exclude ATTITUDE_QUATERNION,HIGHRES_IMU
    python3 mavlink_sniff.py --dash                  # in-place table: latest msg + rate per type

--live and --dash run until Ctrl-C, then print the UART counters.

Requires pymavlink and pyserial (e.g. a venv: python3 -m venv ~/mav &&
~/mav/bin/pip install pymavlink pyserial).
"""
import argparse
import collections
import fcntl
import math
import os
import struct
import sys
import time

from pymavlink import mavutil

TIOCGICOUNT = 0x545D
SCAN_BAUDS = (57600, 115200, 230400, 460800, 921600)

# MAV_SYS_STATUS_SENSOR bits, low 31 only.
SENSOR_BITS = {
    0: "GYRO", 1: "ACCEL", 2: "MAG", 3: "ABS_PRESSURE", 4: "DIFF_PRESSURE",
    5: "GPS", 6: "OPTICAL_FLOW", 7: "VISION_POS", 8: "LASER_POS",
    9: "EXT_GROUND_TRUTH", 10: "ANGULAR_RATE_CTRL", 11: "ATTITUDE_STAB",
    12: "YAW_POS", 13: "Z_ALT_CTRL", 14: "XY_POS_CTRL", 15: "MOTOR_OUTPUTS",
    16: "RC_RECEIVER", 17: "GYRO2", 18: "ACCEL2", 19: "MAG2", 20: "GEOFENCE",
    21: "AHRS", 22: "TERRAIN", 23: "REVERSE_MOTOR", 24: "LOGGING",
    25: "BATTERY", 26: "PROXIMITY", 27: "SATCOM", 28: "PREARM_CHECK",
    29: "OBSTACLE_AVOIDANCE", 30: "PROPULSION",
}
# PX4 custom_mode: main_mode in bits 16-23, sub_mode in bits 24-31.
PX4_MAIN = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO",
            6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE", 9: "SIMPLE", 10: "TERMINATION"}
PX4_AUTO_SUB = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL",
                6: "LAND", 8: "FOLLOW_TARGET", 9: "PRECLAND", 10: "VTOL_TAKEOFF"}


def uart_counters(fd):
    v = struct.unpack("11i", fcntl.ioctl(fd, TIOCGICOUNT, bytes(80))[:44])
    return {"rx": v[4], "tx": v[5], "frame": v[6], "overrun": v[7], "brk": v[9]}


def enum_name(enum, value):
    e = mavutil.mavlink.enums.get(enum, {}).get(value)
    return e.name if e else str(value)


def sensors(mask):
    return [n for b, n in SENSOR_BITS.items() if mask & (1 << b)]


def px4_mode(custom_mode):
    main, sub = (custom_mode >> 16) & 0xFF, (custom_mode >> 24) & 0xFF
    name = PX4_MAIN.get(main, f"main{main}")
    if main == 4:
        name += "." + PX4_AUTO_SUB.get(sub, f"sub{sub}")
    return name


def sniff(port, baud, duration):
    m = mavutil.mavlink_connection(port, baud=baud, autoreconnect=False)
    c0 = uart_counters(m.port.fileno())
    types = collections.Counter()
    last = {}
    bad = 0
    gaps = 0
    last_seq = {}
    t0 = time.time()
    while time.time() - t0 < duration:
        msg = m.recv_msg()
        if msg is None:
            time.sleep(0.002)
            continue
        t = msg.get_type()
        if t == "BAD_DATA":
            bad += 1
            continue
        src = (msg.get_srcSystem(), msg.get_srcComponent())
        seq = msg.get_seq()
        if src in last_seq and seq != (last_seq[src] + 1) % 256:
            gaps += 1
        last_seq[src] = seq
        types[t] += 1
        last[t] = msg
    c1 = uart_counters(m.port.fileno())
    m.close()
    d = {k: c1[k] - c0[k] for k in c0}
    return types, last, bad, gaps, d


def report(port, baud, duration, types, last, bad, gaps, d, verbose=True):
    n = sum(types.values())
    print(f"{port} @ {baud}: {n} msgs in {duration:.0f} s, bad_data={bad}, seq_gaps={gaps}, "
          f"rx={d['rx'] / duration:.0f} B/s, frame_err={d['frame']}, overrun={d['overrun']}, "
          f"brk={d['brk']}, tx={d['tx']}")
    if not verbose or n == 0:
        return
    for t, k in types.most_common():
        print(f"  {t:28s} {k / duration:6.1f} Hz")
    hb = last.get("HEARTBEAT")
    if hb:
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = px4_mode(hb.custom_mode) if hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4 else hb.custom_mode
        print(f"HEARTBEAT  sys={hb.get_srcSystem()} comp={hb.get_srcComponent()} "
              f"{enum_name('MAV_AUTOPILOT', hb.autopilot)} {enum_name('MAV_TYPE', hb.type)} "
              f"mode={mode} status={enum_name('MAV_STATE', hb.system_status)} armed={armed}")
    ss = last.get("SYS_STATUS")
    if ss:
        unhealthy = ss.onboard_control_sensors_enabled & ~ss.onboard_control_sensors_health
        print(f"SYS_STATUS enabled={sensors(ss.onboard_control_sensors_enabled)} "
              f"unhealthy={sensors(unhealthy)} batt={ss.voltage_battery / 1000:.2f} V "
              f"{ss.current_battery / 100:.2f} A remaining={ss.battery_remaining}%")
    a = last.get("ATTITUDE")
    if a:
        print("ATTITUDE   roll=%.1f pitch=%.1f yaw=%.1f deg"
              % tuple(math.degrees(x) for x in (a.roll, a.pitch, a.yaw)))
    es = last.get("EXTENDED_SYS_STATE")
    if es:
        print(f"LANDED     {enum_name('MAV_LANDED_STATE', es.landed_state)}")


def fmt_value(v):
    if isinstance(v, float):
        return "nan" if math.isnan(v) else f"{v:.4g}"
    if isinstance(v, (list, tuple)):
        return "[" + ",".join(fmt_value(x) for x in v[:6]) + (",…]" if len(v) > 6 else "]")
    if isinstance(v, (bytes, bytearray)):
        return v.rstrip(b"\0").decode(errors="replace")
    return str(v)


def summarize(msg):
    """One-line, human-readable form of a message; decoded for the common ones."""
    t = msg.get_type()
    if t == "HEARTBEAT":
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = px4_mode(msg.custom_mode) if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4 else msg.custom_mode
        return (f"{enum_name('MAV_AUTOPILOT', msg.autopilot)} {enum_name('MAV_TYPE', msg.type)} "
                f"mode={mode} status={enum_name('MAV_STATE', msg.system_status)} armed={armed}")
    if t == "ATTITUDE":
        return ("roll=%6.1f pitch=%6.1f yaw=%6.1f deg  rates=%6.2f %6.2f %6.2f rad/s"
                % (math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw),
                   msg.rollspeed, msg.pitchspeed, msg.yawspeed))
    if t == "SYS_STATUS":
        unhealthy = msg.onboard_control_sensors_enabled & ~msg.onboard_control_sensors_health
        return (f"batt={msg.voltage_battery / 1000:.2f} V {msg.current_battery / 100:.2f} A "
                f"{msg.battery_remaining}% load={msg.load / 10:.1f}% unhealthy={sensors(unhealthy)}")
    if t == "EXTENDED_SYS_STATE":
        return enum_name("MAV_LANDED_STATE", msg.landed_state)
    if t == "STATUSTEXT":
        return f"[{enum_name('MAV_SEVERITY', msg.severity)}] {fmt_value(msg.text)}"
    return " ".join(f"{f}={fmt_value(getattr(msg, f))}" for f in msg.get_fieldnames())


def stream(port, baud, live, only, exclude):
    """Receive until Ctrl-C. live=True prints each message; otherwise redraws a dashboard."""
    m = mavutil.mavlink_connection(port, baud=baud, autoreconnect=False)
    fd = m.port.fileno()
    c0 = uart_counters(fd)
    counts = collections.Counter()
    window = collections.defaultdict(collections.deque)  # type -> arrival times in last 2 s
    last = {}
    bad = 0
    t0 = time.time()
    next_draw = 0.0
    try:
        while True:
            msg = m.recv_msg()
            now = time.time()
            if msg is None:
                time.sleep(0.002)
            else:
                t = msg.get_type()
                if t == "BAD_DATA":
                    bad += 1
                elif (not only or t in only) and t not in exclude:
                    counts[t] += 1
                    last[t] = msg
                    window[t].append(now)
                    if live:
                        print(f"{now - t0:9.3f} {msg.get_srcSystem():>3}:{msg.get_srcComponent():<3} "
                              f"{t:26s} {summarize(msg)}", flush=True)
            if not live and now >= next_draw:
                next_draw = now + 0.25
                cols = 200
                try:
                    cols = os.get_terminal_size().columns
                except OSError:
                    pass
                lines = [f"{port} @ {baud}  up {now - t0:6.1f} s  msgs={sum(counts.values())} "
                         f"bad_data={bad}  (Ctrl-C to stop)", ""]
                for t in sorted(last):
                    w = window[t]
                    while w and now - w[0] > 2.0:
                        w.popleft()
                    age = now - (w[-1] if w else t0)
                    lines.append(f"{t:26s} {len(w) / 2.0:6.1f} Hz {age:5.1f}s  {summarize(last[t])}"[:cols])
                sys.stdout.write("\x1b[H\x1b[2J" + "\n".join(lines) + "\n")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        c1 = uart_counters(fd)
        m.close()
        d = {k: c1[k] - c0[k] for k in c0}
        print(f"\nstopped after {time.time() - t0:.1f} s: {sum(counts.values())} msgs, bad_data={bad}, "
              f"rx={d['rx']} B, frame_err={d['frame']}, overrun={d['overrun']}, tx={d['tx']}",
              file=sys.stderr)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", default="/dev/ttyAMA0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("-t", "--duration", type=float, default=10.0)
    p.add_argument("--scan", action="store_true", help="try common baud rates, 5 s each")
    p.add_argument("--live", action="store_true", help="print each message as it arrives")
    p.add_argument("--dash", action="store_true", help="refreshing per-type table")
    p.add_argument("--types", default="", help="comma list: only show these message types")
    p.add_argument("--exclude", default="", help="comma list: hide these message types")
    args = p.parse_args()
    only = {s.strip().upper() for s in args.types.split(",") if s.strip()}
    exclude = {s.strip().upper() for s in args.exclude.split(",") if s.strip()}
    if args.live or args.dash:
        stream(args.port, args.baud, args.live, only, exclude)
    elif args.scan:
        for baud in SCAN_BAUDS:
            report(args.port, baud, 5.0, *sniff(args.port, baud, 5.0), verbose=False)
    else:
        report(args.port, args.baud, args.duration, *sniff(args.port, args.baud, args.duration))


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:  # output piped into head/grep that exited
        sys.stdout = open(os.devnull, "w")
