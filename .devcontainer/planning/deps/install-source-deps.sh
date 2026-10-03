#!/bin/bash
# Source and binary dependencies for the planning component, under /opt/castor/third_party.
#
#   ONNX Runtime  prebuilt CPU release. On the Pi 5 benchmark (2026-09-29) ORT
#                 fp32 single-thread ran the 135-input policy in ~0.34 ms, about
#                 nine times faster than PyTorch's aarch64 wheel.
set -euo pipefail

ORT_VERSION="${ORT_VERSION:-1.30.0}"
PREFIX=/opt/castor/third_party
ARCH="${TARGETARCH:-$(dpkg --print-architecture)}"

case "$ARCH" in
    amd64) ort_arch=x64;     ort_sha256=a5ed5a3cac51fbb2e90da632ae43d19212faaa20e76484e62bcb7c23ddb3b3fd ;;
    arm64) ort_arch=aarch64; ort_sha256=e16a27a8ed330bbc698df7330b0cf56e722f354e3bcc92118682c74ef3c3e3da ;;
    *) echo "ONNX Runtime: unsupported architecture $ARCH" >&2; exit 1 ;;
esac
[ "$ORT_VERSION" = "1.30.0" ] || ort_sha256=""   # checksums above are for 1.30.0 only

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
mkdir -p "$PREFIX" "$PREFIX/env.d"

echo "==> ONNX Runtime $ORT_VERSION ($ort_arch)"
tgz="onnxruntime-linux-${ort_arch}-${ORT_VERSION}.tgz"
curl -fsSL --retry 10 --retry-all-errors --retry-delay 3 -C - -o "$work/$tgz" \
    "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/$tgz"
if [ -n "$ort_sha256" ]; then
    echo "$ort_sha256  $work/$tgz" | sha256sum -c -
else
    echo "    (no pinned checksum for $ORT_VERSION; not verified)"
fi
tar -xzf "$work/$tgz" -C "$work"
rm -rf "$PREFIX/onnxruntime"
mv "$work/onnxruntime-linux-${ort_arch}-${ORT_VERSION}" "$PREFIX/onnxruntime"

echo "==> planning source deps installed under $PREFIX"
du -sh "$PREFIX"/* 2>/dev/null || true
