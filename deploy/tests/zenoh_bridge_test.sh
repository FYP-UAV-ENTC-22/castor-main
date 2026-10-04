#!/bin/bash
# End-to-end test of the zenoh bridge between a drone and the ground station.
#
#   deploy/tests/zenoh_bridge_test.sh --local                 everything on this machine
#   deploy/tests/zenoh_bridge_test.sh --remote drone1@192.168.1.8 [--multicast]
#
# --local   two DDS domains on one host stand in for two hosts: the ground station
#           (domain 20, bridge on 7447) and drone2 (domain 21, bridge on 7448,
#           connecting to 7447). Different domains keep the DDS graphs apart, so
#           zenoh is the only path between them.
# --remote  the drone is a real Pi running the deployed stack (compose project
#           "castor"); this machine is the ground station (domain 20, bridge on
#           7447). The ground station connects to the Pi explicitly, or, with
#           --multicast, finds it by multicast scouting.
#
# Checks: 0 isolation; 1 allow-listed state crosses; 2 a non-listed topic is seen
# by the bridge but not routed; 3 a near-miss of an allowed name doesn't cross
# (anchoring); 4 odometry is capped at 20 Hz across, each sample once; 5 /team/command
# reaches the drone; 6 state still crosses after two restarts and a crash of the ros2
# launch process (the bridge keys routes by node name: zenoh-plugin-ros2dds #702);
# 7 after those restarts and a ground-station reconnect, each state sample still
# crosses once (KNOWN, not a failure, with bridge 1.10.1: zenoh-plugin-ros2dds #722).
# --remote restarts the drone's bridge at the end to drop the stale routes 6 leaves.
#
# Environment: CASTOR_TAG (default local for --local, main for --remote),
# CASTOR_REGISTRY, ZENOH_BRIDGE_IMAGE. Uses only the system image and the bridge.
set -uo pipefail

MODE="" HOST="" MULTICAST=0
while [ $# -gt 0 ]; do
    case "$1" in
        --local) MODE=local; shift ;;
        --remote) MODE=remote; HOST="$2"; shift 2 ;;
        --multicast) MULTICAST=1; shift ;;
        -h|--help) sed -n '2,26p' "$0" | sed 's/^# \?//'; exit 0 ;;
        *) echo "unknown option '$1'" >&2; exit 2 ;;
    esac
done
[ -n "$MODE" ] || { sed -n '2,26p' "$0" | sed 's/^# \?//'; exit 2; }

REG="${CASTOR_REGISTRY:-ghcr.io/fyp-uav-entc-22}"
TAG="${CASTOR_TAG:-$([ "$MODE" = local ] && echo local || echo main)}"
IMG="$REG/castor-system:$TAG"
BRIDGE="${ZENOH_BRIDGE_IMAGE:-ghcr.io/fyp-uav-entc-22/zenoh-bridge-ros2dds:1.10.1}"
CY='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="127.0.0.1" multicast="true"/></Interfaces><DontRoute>true</DontRoute></General></Domain></CycloneDDS>'
S="$(mktemp -d "${TMPDIR:-/tmp}/castor-zenoh-test.XXXXXX")"
mkdir -p "$S/gcs-run" "$S/run"
pass=0 fail=0 known=""
ok()  { echo "PASS: $*"; pass=$((pass+1)); }
bad() { echo "FAIL: $*"; fail=$((fail+1)); }
known() { echo "KNOWN: $*"; known=$((known+1)); }   # an upstream defect, reported but not failed
strip() { sed 's/\x1b\[[0-9;]*m//g'; }
# wait_for <seconds> <command...>: poll until the command succeeds
wait_for() { local t=$1; shift; for _ in $(seq 1 "$t"); do "$@" && return 0; sleep 1; done; return 1; }

if [ "$MODE" = local ]; then
    NS=drone2 D_SYS=zt-drone2-system D_BRIDGE=zt-drone2-bridge
    on_drone() { bash -c "$1"; }
else
    NS="$(ssh "$HOST" "docker exec castor-system-1 bash -c 'source /opt/ros/jazzy/setup.bash && source /opt/castor/common/setup.bash && castor-config env'" | sed -n 's/^export CASTOR_NS=//p')"
    D_SYS=castor-system-1 D_BRIDGE=castor-zenoh-bridge-1
    # shellcheck disable=SC2029  # the command is built here on purpose and runs on the drone
    on_drone() { ssh "$HOST" "$1"; }
    [ -n "$NS" ] || { echo "no CASTOR stack running on $HOST" >&2; exit 2; }
fi
drone_ros() { on_drone "docker exec $D_SYS bash -c 'source /opt/castor/castor_env.sh; $1'"; }
gcs_ros()   { docker run --rm --network host --ipc host -v "$S/gcs.yaml:/etc/castor/robot.yaml:ro" "$IMG" \
                  bash -c "source /opt/castor/castor_env.sh; $1"; }

cleanup() {
    docker rm -f zt-gcs-bridge >/dev/null 2>&1 || true
    [ "$MODE" = local ] && docker rm -f "$D_SYS" "$D_BRIDGE" >/dev/null 2>&1
    on_drone "docker exec $D_SYS bash -c 'kill \$(cat /tmp/zt-pub.pid 2>/dev/null) 2>/dev/null'" >/dev/null 2>&1 || true
    # the restarts in 6 leave stale routes on the drone's bridge (#722, see 7); a bridge restart drops them
    [ "$MODE" = remote ] && on_drone "docker restart $D_BRIDGE" >/dev/null 2>&1
    rm -rf "$S"
}
trap cleanup EXIT

# ---------------------------------------------------------------- ground station
if [ "$MODE" = remote ] && [ "$MULTICAST" -eq 0 ]; then
    connect="[\"tcp/${HOST#*@}:7447\"]"; mcast=false
elif [ "$MODE" = remote ]; then
    connect="[]"; mcast=true
else
    connect="[]"; mcast=false
fi
cat > "$S/gcs.yaml" <<EOF
robot: {id: 100, namespace: gcs, hardware: laptop}
team: {size: 1, index: 0}
ros: {domain_id: 20}
fc: {enabled: false}
zenoh: {role: ground_station, connect: $connect, listen_port: 7447, multicast_scouting: $mcast}
EOF
render() {  # <robot.yaml> <run dir>
    docker run --rm --network host -v "$1:/etc/castor/robot.yaml:ro" -v "$2:/run/castor" "$IMG" \
        castor-config zenoh-bridge --out /run/castor/zenoh-bridge.json5 >/dev/null
}
render "$S/gcs.yaml" "$S/gcs-run"
docker run -d --name zt-gcs-bridge --network host -e ROS_DISTRO=jazzy -e CYCLONEDDS_URI="$CY" \
    -v "$S/gcs-run:/run/castor:ro" "$BRIDGE" -c /run/castor/zenoh-bridge.json5 >/dev/null

# ---------------------------------------------------------------- drone (local only)
if [ "$MODE" = local ]; then
    cat > "$S/robot.yaml" <<EOF
robot: {id: 2, namespace: drone2, hardware: laptop}
team: {size: 3, index: 1}
ros: {domain_id: 21}
fc: {enabled: false}
zenoh: {role: drone, connect: ["tcp/127.0.0.1:7447"], listen_port: 7448, multicast_scouting: false}
EOF
    render "$S/robot.yaml" "$S/run"
    docker run -d --name "$D_BRIDGE" --network host -e ROS_DISTRO=jazzy -e RUST_LOG=zenoh_plugin_ros2dds=debug \
        -e CYCLONEDDS_URI="$CY" -v "$S/run:/run/castor:ro" "$BRIDGE" -c /run/castor/zenoh-bridge.json5 >/dev/null
    sleep 2   # bridges first, like a Pi at boot
    docker run -d --name "$D_SYS" --restart unless-stopped --init --network host --ipc host --pid host \
        --stop-signal SIGINT -v "$S/robot.yaml:/etc/castor/robot.yaml:ro" -v "$S/run:/run/castor" "$IMG" >/dev/null
fi
echo "drone /$NS ($MODE${HOST:+ $HOST}), ground station on this host, images $IMG + $BRIDGE"

state_arrives() { gcs_ros "timeout 8 ros2 topic echo --once /$NS/system/state castor_interfaces/msg/SupervisorState" 2>/dev/null | grep -q '^state: '; }

# 0. isolation (meaningful locally, where both graphs share one host)
if [ "$MODE" = local ]; then
    wait_for 30 drone_ros "ros2 node list --no-daemon | grep -q /$NS/" >/dev/null 2>&1
    nodes=$(gcs_ros "ros2 node list --no-daemon" 2>&1)
    if echo "$nodes" | grep -q "/$NS/"; then bad "0 $NS nodes visible on the ground station's DDS graph"
    else ok "0 $NS's DDS graph is invisible to the ground station"; fi
fi

# 1. allow-listed topic crosses
if wait_for 6 state_arrives; then ok "1 /$NS/system/state reaches the ground station"
else bad "1 no /$NS/system/state at the ground station"; fi

# 2 and 3. a non-listed topic and a near-miss of an allowed name stay on the drone
drone_ros "ros2 topic pub -r 5 /$NS/secret/data std_msgs/msg/String \"{data: leaked}\" >/dev/null 2>&1 & echo \$! > /tmp/zt-pub.pid
           ros2 topic pub -r 5 /$NS/vehicle/odom_raw std_msgs/msg/String \"{data: leaked}\" >/dev/null 2>&1 & echo \$! >> /tmp/zt-pub.pid"
sleep 4
local_seen=$(drone_ros "timeout 6 ros2 topic echo --once /$NS/secret/data std_msgs/msg/String" 2>&1 | grep -c leaked)
for t in secret/data vehicle/odom_raw; do
    n=$([ "$t" = secret/data ] && echo 2 || echo 3)
    out=$(gcs_ros "timeout 8 ros2 topic echo --once /$NS/$t std_msgs/msg/String" 2>&1)
    if [ "$local_seen" -ge 1 ] && ! echo "$out" | grep -q leaked; then ok "$n /$NS/$t stays on the drone"
    else bad "$n /$NS/$t: local=$local_seen, at the ground station: $(echo "$out" | tail -1)"; fi
done
if [ "$MODE" = local ]; then
    docker logs "$D_BRIDGE" 2>&1 | strip | grep -q "declares Publisher /$NS/secret/data.*Denied per config" \
        && echo "    (bridge log: /$NS/secret/data Denied per config)"
fi

# stamps <topic> <type> <seconds>: header stamps received at the ground station, one "sec,nanosec" per line
stamps() { gcs_ros "timeout $3 ros2 topic echo --csv --field header.stamp $1 $2" 2>/dev/null | grep ','; }
# rate_dups <file>: "<Hz of distinct samples> <copies per sample>"
rate_dups() {
    sort -u "$1" | awk -F, -v all="$(grep -c , "$1")" '
        NR == 1 {t0 = $1 + $2 / 1e9} {t = $1 + $2 / 1e9; n++}
        END {printf "%.1f %.2f\n", (n > 1 && t > t0) ? (n - 1) / (t - t0) : 0, n ? all / n : 0}'
}

# 4. odometry rate cap, every sample once (stamped, so a sample arriving twice is visible)
drone_ros "ros2 topic pub -r 50 /$NS/vehicle/odom nav_msgs/msg/Odometry \"{header: auto}\" >/dev/null 2>&1 & echo \$! >> /tmp/zt-pub.pid"
sleep 4
stamps "/$NS/vehicle/odom" nav_msgs/msg/Odometry 12 > "$S/odom.csv"
read -r hz copies < <(rate_dups "$S/odom.csv")
if awk -v h="$hz" -v c="$copies" 'BEGIN{exit !(h > 5 && h <= 21 && c == 1)}'; then ok "4 odom crosses at ${hz} Hz (published at 50, cap 20), each sample once"
elif awk -v c="$copies" 'BEGIN{exit !(c > 1.05)}'; then
    bad "4 odom: each sample arrives ${copies}x (${hz} Hz distinct): stale routes on the drone's bridge, zenoh-plugin-ros2dds #722"
else bad "4 odom rate across: '${hz}' Hz"; fi
drone_ros "kill \$(cat /tmp/zt-pub.pid) 2>/dev/null; rm -f /tmp/zt-pub.pid" >/dev/null 2>&1

# 5. team command reaches the drone
# Detached listener; 16 messages at 2 Hz, so the drone side has time to match the new route (5 at 1 Hz was
# measured to miss on a real Pi).
on_drone "docker exec -d $D_SYS bash -c 'source /opt/castor/castor_env.sh; timeout 40 ros2 topic echo --once /team/command castor_interfaces/msg/TeamCommand > /tmp/zt-cmd.txt 2>&1'"
sleep 4
gcs_ros "timeout 20 ros2 topic pub -r 2 -t 16 -w 1 /team/command castor_interfaces/msg/TeamCommand '{command: status, robot_ids: []}'" >/dev/null 2>&1
if wait_for 10 drone_ros "grep -q \"command: status\" /tmp/zt-cmd.txt"; then ok "5 /team/command reaches the drone"
else bad "5 the drone did not get /team/command"; fi

# 6. restarts and a crash, past the old participants' lease
docker_on_drone() { on_drone "docker $*"; }
docker_on_drone restart "$D_SYS" >/dev/null; sleep 1; docker_on_drone restart "$D_SYS" >/dev/null
sleep 15
# The exact ros2 launch process (pid: host means a pattern match could hit other processes, so never
# pkill -f): the topmost "ros2 launch" in the container, i.e. one whose parent isn't another one. Whether
# docker top lists docker-init above it differs between Docker versions (it does on the laptop's, not on
# the Pi's 29.8), so don't rely on it.
launch_pid=$(on_drone "docker top $D_SYS -o pid,ppid,args" | awk '
    NR > 1 && /bin\/ros2 launch / {pid[NR] = $1; ppid[NR] = $2; is[$1] = 1}
    END {for (r in pid) if (!(ppid[r] in is)) {print pid[r]; exit}}')
restarts=$(on_drone "docker inspect -f '{{.RestartCount}}' $D_SYS")
if [ -n "$launch_pid" ]; then
    on_drone "docker exec $D_SYS kill -KILL $launch_pid" >/dev/null 2>&1
    wait_for 30 bash -c "[ \"\$($( [ "$MODE" = remote ] && echo "ssh $HOST ")docker inspect -f '{{.RestartCount}}' $D_SYS)\" -gt $restarts ]"
    echo "    crashed ros2 launch (pid $launch_pid); container restarted by its policy"
else
    bad "6 could not find the ros2 launch process in $D_SYS"
fi
sleep 15
if wait_for 20 state_arrives; then ok "6 /$NS/system/state still crosses after 2 restarts and a crash"
else bad "6 /$NS/system/state stopped crossing after the restarts"; fi

# 7. the restarts removed and re-created the state route on the drone's bridge. In zenoh-plugin-ros2dds
# 1.10.1 the removed route's matching listener survives (#722, fix in PR #738) and re-creates a DDS reader
# when the ground station reconnects, so each sample then crosses more than once. KNOWN while the bridge
# image lacks the fix; it turns into a PASS once it has it.
docker restart zt-gcs-bridge >/dev/null
wait_for 20 state_arrives
stamps "/$NS/system/state" castor_interfaces/msg/SupervisorState 10 > "$S/state.csv"
read -r hz copies < <(rate_dups "$S/state.csv")
if awk -v h="$hz" -v c="$copies" 'BEGIN{exit !(h > 0 && c == 1)}'; then ok "7 after a ground-station reconnect each state sample crosses once"
elif awk -v c="$copies" 'BEGIN{exit !(c > 1.05)}'; then
    known "7 after a ground-station reconnect each state sample crosses ${copies}x (zenoh-plugin-ros2dds #722, bridge 1.10.1)"
else bad "7 no /$NS/system/state after the ground station reconnected"; fi

echo "passed $pass, failed $fail${known:+, known $known}"
[ "$fail" -eq 0 ]
