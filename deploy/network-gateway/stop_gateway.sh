#!/bin/bash
# Reverses base_station_setup.sh on THIS machine: removes the NAT/FORWARD
# rules that script added, so this laptop stops acting as an internet
# gateway for FFT. Run with sudo, on the machine you want to stop being
# the gateway.
#
# Note: this only matters if you actively want to stop sharing (e.g.
# handing the laptop off, or don't want FFT devices using your internet
# anymore). It is NOT required for safety - if you just unplug your
# internet source or disconnect from FFT instead, everything already fails
# closed on its own (see README).
#
# Leaves net.ipv4.ip_forward=1 in place - harmless with no NAT/FORWARD
# rules to act on, and re-enabling it every session is just friction.
set -euo pipefail

COMMENT="drone-ops-fft-gateway"

delete_tagged_rules() {
    local table="$1" chain="$2" line del found=0
    while line=$(iptables -t "$table" -S "$chain" 2>/dev/null | grep -- "--comment $COMMENT" | head -n1) && [ -n "$line" ]; do
        del="-D ${line#-A }"
        # shellcheck disable=SC2086
        iptables -t "$table" $del
        found=1
    done
    [ "$found" = 1 ] && echo "Removed $chain rule(s)." || echo "No $chain rules found (already stopped, or never started)."
}

echo "=== Removing FFT gateway NAT/FORWARD rules ==="
delete_tagged_rules nat POSTROUTING
delete_tagged_rules filter FORWARD

echo
echo "=== Persisting (so it stays off across reboot) ==="
if command -v netfilter-persistent >/dev/null 2>&1; then
    netfilter-persistent save
    echo "Saved."
else
    echo "netfilter-persistent not installed - nothing to persist (rules were never saved either)."
fi

echo
echo "This machine is no longer acting as an FFT gateway."
echo "Drones/teammates that were pointed at it will simply lose internet until re-pointed elsewhere."
