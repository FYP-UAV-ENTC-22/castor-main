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
if curl -fsSL --retry 2 --retry-delay 2 -o "$work/$tgz" \
    "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/$tgz"; then
    if [ -n "$ort_sha256" ]; then
        echo "$ort_sha256  $work/$tgz" | sha256sum -c -
    else
        echo "    (no pinned checksum for $ORT_VERSION; not verified)"
    fi
    tar -xzf "$work/$tgz" -C "$work"
    rm -rf "$PREFIX/onnxruntime"
    mv "$work/onnxruntime-linux-${ort_arch}-${ORT_VERSION}" "$PREFIX/onnxruntime"
else
    echo "    GitHub release unavailable; falling back to NuGet microsoft.ml.onnxruntime $ORT_VERSION"
    nuget_arch="$ort_arch"
    [ "$ort_arch" = "aarch64" ] && nuget_arch="arm64"
    nupkg="$work/ort.nupkg"
    # --retry alone does not cover a transfer that dies mid-stream (curl error 18), which this ~170 MB
    # download does often enough to break builds; --retry-all-errors retries that too.
    curl -fsSL --retry 5 --retry-delay 2 --retry-all-errors -o "$nupkg" \
        "https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime/${ORT_VERSION}/microsoft.ml.onnxruntime.${ORT_VERSION}.nupkg"
    python3 -m zipfile -e "$nupkg" "$work/nupkg"
    rm -rf "$PREFIX/onnxruntime"
    mkdir -p "$PREFIX/onnxruntime/include" "$PREFIX/onnxruntime/lib"
    cp -a "$work/nupkg/build/native/include/." "$PREFIX/onnxruntime/include/"
    cp -a "$work/nupkg/runtimes/linux-${nuget_arch}/native/." "$PREFIX/onnxruntime/lib/"
    # NuGet ships the library as a plain libonnxruntime.so, but its SONAME is libonnxruntime.so.1, so that is the
    # name the loader looks for: without it anything linked against ORT dies with "cannot open shared object file".
    # The GitHub release ships both links; here we add them (major from ORT_VERSION, e.g. 1.30.0 -> 1).
    if [ -f "$PREFIX/onnxruntime/lib/libonnxruntime.so" ]; then
        for suffix in "${ORT_VERSION%%.*}" "$ORT_VERSION"; do
            [ -e "$PREFIX/onnxruntime/lib/libonnxruntime.so.$suffix" ] \
                || ln -s libonnxruntime.so "$PREFIX/onnxruntime/lib/libonnxruntime.so.$suffix"
        done
    fi
fi

echo "==> planning source deps installed under $PREFIX"
du -sh "$PREFIX"/* 2>/dev/null || true
