#!/bin/bash
# Source and binary dependencies for the vehicle component, installed under
# /opt/castor/third_party so the runtime image can copy them in one step.
#
#   Micro XRCE-DDS Agent  the PX4 uXRCE-DDS bridge. v2.4.3 is what PX4 documents
#                         for ROS 2 Jazzy (the FC's client is not built for DDS v3).
#   mavlink-router        FC MAVLink port -> UDP, so QGC can share the link.
#   QGroundControl        extracted AppImage, opened on demand with castor-qgc.
#
# Everything is fetched first and compiled after, so a dropped connection fails
# the step in seconds instead of after the compiles.
set -euo pipefail

XRCE_AGENT_VERSION="${XRCE_AGENT_VERSION:-v2.4.3}"
MAVLINK_ROUTER_VERSION="${MAVLINK_ROUTER_VERSION:-v4}"
QGC_VERSION="${QGC_VERSION:-v5.1.4}"
PREFIX=/opt/castor/third_party
ARCH="${TARGETARCH:-$(dpkg --print-architecture)}"
JOBS="$(nproc)"

mkdir -p "$PREFIX" "$PREFIX/env.d"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

retry() {
    local n
    for n in 1 2 3 4 5; do
        "$@" && return 0
        echo "    attempt $n failed: $*" >&2
        sleep $((n * 5))
    done
    return 1
}
fetch() { curl -fsSL --retry 10 --retry-all-errors --retry-delay 3 -C - -o "$1" "$2"; }
clone() { rm -rf "$2"; git clone --quiet --depth 1 --branch "$3" "${@:4}" "$1" "$2"; }

case "$ARCH" in
    amd64) qarch=x86_64 ;;
    arm64) qarch=aarch64 ;;
    *) echo "unsupported architecture $ARCH" >&2; exit 1 ;;
esac

echo "==> fetching sources"
retry fetch "$work/qgc.AppImage" \
    "https://github.com/mavlink/qgroundcontrol/releases/download/${QGC_VERSION}/QGroundControl-${qarch}.AppImage"
retry clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$work/agent" "$XRCE_AGENT_VERSION"
retry clone https://github.com/mavlink-router/mavlink-router.git "$work/mavlink-router" "$MAVLINK_ROUTER_VERSION" \
    --recurse-submodules --shallow-submodules

echo "==> QGroundControl $QGC_VERSION ($ARCH)"
chmod +x "$work/qgc.AppImage"
(cd "$work" && ./qgc.AppImage --appimage-extract >/dev/null)
rm -rf "$PREFIX/qgc"
mv "$work/squashfs-root" "$PREFIX/qgc"
chmod -R a+rX "$PREFIX/qgc"
rm -f "$work/qgc.AppImage"

echo "==> mavlink-router $MAVLINK_ROUTER_VERSION"
meson setup "$work/mavlink-router/build" "$work/mavlink-router" \
    --prefix="$PREFIX/mavlink-router" --buildtype=release \
    -Dsystemdsystemunitdir="$work/unused-systemd" \
    -Dcpp_std=gnu++14   # v4 defaults to gnu++11; Ubuntu 24.04's gtest (picked up for its tests) needs C++14
ninja -C "$work/mavlink-router/build" install
cat >"$PREFIX/env.d/mavlink-router.sh" <<EOF
export PATH="$PREFIX/mavlink-router/bin:\$PATH"
EOF

echo "==> Micro XRCE-DDS Agent $XRCE_AGENT_VERSION"
# The superbuild downloads Fast DDS / Fast CDR / micro-CDR sources itself.
cmake -S "$work/agent" -B "$work/agent/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$PREFIX/xrce-agent" \
    -DUAGENT_BUILD_TESTS=OFF -DUAGENT_BUILD_EXAMPLES=OFF
retry cmake --build "$work/agent/build" -j "$JOBS"
cmake --install "$work/agent/build"
missing=$(LD_LIBRARY_PATH="$PREFIX/xrce-agent/lib" ldd "$PREFIX/xrce-agent/bin/MicroXRCEAgent" | grep "not found" || true)
[ -z "$missing" ] || { echo "MicroXRCEAgent has unresolved libraries:"$'\n'"$missing" >&2; exit 1; }

echo "==> vehicle source deps installed under $PREFIX"
du -sh "$PREFIX"/* 2>/dev/null || true
