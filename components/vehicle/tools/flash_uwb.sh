#!/bin/bash
# Flash a UWB firmware build onto an STM32 board over its ST-Link.
#
#   tools/flash_uwb.sh <firmware.elf|.bin> [--target stm32f4x|stm32h7x] [--serial <st-link serial>]
#
# --target defaults to stm32f4x (Nucleo-F446RE); use stm32h7x for the H753ZI.
# Runs on the host or inside the vehicle container, which ships openocd and
# stlink-tools for amd64 and arm64, so it works on the Pi too. Building the
# firmware still happens on a dev machine: it needs the Qorvo DW3_QM33_SDK,
# which is gitignored and never goes into an image. See
# components/vehicle/uwb_firmware/firmware/README.md for the build.
set -euo pipefail

usage() { sed -n '2,12p' "$0" | sed 's/^# \?//'; exit "${1:-0}"; }

[ $# -ge 1 ] || usage 2
case "$1" in -h|--help) usage ;; esac
FW="$1"; shift
TARGET=stm32f4x
SERIAL=""
while [ $# -gt 0 ]; do
    case "$1" in
        --target) TARGET="$2"; shift 2 ;;
        --serial) SERIAL="$2"; shift 2 ;;
        -h|--help) usage ;;
        *) echo "flash_uwb.sh: unknown option '$1'" >&2; usage 2 ;;
    esac
done
[ -f "$FW" ] || { echo "flash_uwb.sh: no such file: $FW" >&2; exit 1; }
case "$TARGET" in stm32f4x|stm32h7x) ;; *) echo "flash_uwb.sh: --target must be stm32f4x or stm32h7x" >&2; exit 2 ;; esac

case "$FW" in
    *.elf)
        command -v openocd >/dev/null || { echo "flash_uwb.sh: openocd not found" >&2; exit 1; }
        extra=()
        [ -n "$SERIAL" ] && extra+=(-c "adapter serial $SERIAL")
        exec openocd -f interface/stlink.cfg "${extra[@]}" -f "target/$TARGET.cfg" \
            -c "program $FW verify reset exit"
        ;;
    *.bin)
        command -v st-flash >/dev/null || { echo "flash_uwb.sh: st-flash not found" >&2; exit 1; }
        args=(--reset)
        [ -n "$SERIAL" ] && args+=(--serial "$SERIAL")
        exec st-flash "${args[@]}" write "$FW" 0x08000000
        ;;
    *)
        echo "flash_uwb.sh: expected a .elf or .bin, got $FW" >&2; exit 1 ;;
esac
