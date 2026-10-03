#!/bin/bash
# Copy PX4's message definitions into components/vehicle/px4_msgs, the same way
# PX4's own Tools/copy_to_ros_ws.sh and px4_msgs sync job do: msg/*.msg plus
# msg/versioned/*.msg into msg/, srv/*.srv (and srv/versioned) into srv/.
#
# The companion's px4_msgs must match the firmware exactly, so it always comes
# from the PX4-Autopilot submodule at its pinned commit. msg/, srv/ and
# PX4_COMMIT are gitignored; run this after moving the submodule.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4="$ROOT/components/vehicle/PX4-Autopilot"
DST="$ROOT/components/vehicle/px4_msgs"

[ -d "$PX4/msg" ] || {
    echo "sync_px4_msgs: $PX4/msg not found. Check out the submodule:" >&2
    echo "    git submodule update --init --depth 1 components/vehicle/PX4-Autopilot" >&2
    exit 1
}

rm -rf "$DST/msg" "$DST/srv"
mkdir -p "$DST/msg" "$DST/srv"
cp "$PX4"/msg/*.msg "$DST/msg/"
compgen -G "$PX4/msg/versioned/*.msg" >/dev/null && cp "$PX4"/msg/versioned/*.msg "$DST/msg/"
cp "$PX4"/srv/*.srv "$DST/srv/"
compgen -G "$PX4/srv/versioned/*.srv" >/dev/null && cp "$PX4"/srv/versioned/*.srv "$DST/srv/"
git -C "$PX4" rev-parse HEAD > "$DST/PX4_COMMIT"

printf 'px4_msgs: %s msgs, %s srvs from PX4 %s\n' \
    "$(ls "$DST/msg" | wc -l)" "$(ls "$DST/srv" | wc -l)" "$(cut -c1-10 "$DST/PX4_COMMIT")"
