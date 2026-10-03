#!/bin/bash
# Turns THIS machine into an internet gateway for drones on the FFT wifi
# network (which has no internet of its own): NAT + IP forwarding, bridging
# the FFT interface to whichever interface currently has real internet.
#
# Portable / safe for any teammate's laptop to run:
#   - Auto-detects the FFT interface (by SSID, not assumed device name).
#   - Auto-detects the internet interface by actually testing connectivity,
#     not just checking for a default route (FFT's own dead-end DHCP
#     gateway has one of those too).
#   - If THIS machine has no working internet right now - e.g. everyone's
#     on FFT and someone else currently has the dongle/hotspot - it aborts
#     cleanly instead of setting up broken or dangerous NAT rules. Just run
#     it on whichever teammate's laptop currently has internet.
#   - Re-running (e.g. after switching from ethernet to a phone hotspot) is
#     safe: it removes its own previously-added rules (tagged with a
#     comment) before adding fresh ones, so nothing stale accumulates.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/detect_fft.sh"

COMMENT="drone-ops-fft-gateway"

echo "=== Detecting FFT interface (SSID: $FFT_SSID) ==="
FFT_IF=$(detect_fft_interface) || {
    echo "ERROR: not connected to the '$FFT_SSID' wifi network. Connect to FFT first." >&2
    exit 1
}
echo "FFT interface: $FFT_IF ($(fft_interface_ip "$FFT_IF"))"

echo
echo "=== Detecting a working internet interface ==="
WAN_IF=$(detect_internet_interface "$FFT_IF") || {
    echo "ERROR: no other interface on this machine has working internet right now." >&2
    echo "This machine can't act as the drone gateway at the moment - that's fine," >&2
    echo "just run this script on whichever teammate's laptop currently has internet." >&2
    exit 1
}
if [ "$WAN_IF" = "$FFT_IF" ]; then
    echo "ERROR: detected internet interface is the same as the FFT interface - refusing to proceed." >&2
    exit 1
fi
echo "Internet interface: $WAN_IF"

echo
echo "=== IP forwarding ==="
sysctl -w net.ipv4.ip_forward=1
if grep -qE '^\s*net\.ipv4\.ip_forward\s*=' /etc/sysctl.conf; then
    sed -i 's/^\s*net\.ipv4\.ip_forward\s*=.*/net.ipv4.ip_forward=1/' /etc/sysctl.conf
else
    echo "net.ipv4.ip_forward=1" >> /etc/sysctl.conf
fi
sysctl -p /etc/sysctl.conf >/dev/null
grep '^net.ipv4.ip_forward' /etc/sysctl.conf

echo
echo "=== ufw check ==="
if command -v ufw >/dev/null 2>&1 && ufw status | grep -q "Status: active"; then
    echo "ufw is active -> setting default forward policy to ACCEPT and adding route rule"
    sed -i 's/^DEFAULT_FORWARD_POLICY=.*/DEFAULT_FORWARD_POLICY="ACCEPT"/' /etc/default/ufw
    ufw route allow in on "$FFT_IF" out on "$WAN_IF" || true
    ufw reload
else
    echo "ufw not active, skipping ufw-specific config"
fi

echo
echo "=== iptables NAT/FORWARD rules ==="
# Remove any rules this script previously added (any interface pair), so
# switching WAN interfaces between runs doesn't leave stale rules behind.
delete_tagged_rules() {
    local table="$1" chain="$2" line del
    while line=$(iptables -t "$table" -S "$chain" 2>/dev/null | grep -- "--comment $COMMENT" | head -n1) && [ -n "$line" ]; do
        del="-D ${line#-A }"
        # shellcheck disable=SC2086
        iptables -t "$table" $del
    done
}
delete_tagged_rules nat POSTROUTING
delete_tagged_rules filter FORWARD

iptables -t nat -A POSTROUTING -o "$WAN_IF" -m comment --comment "$COMMENT" -j MASQUERADE
iptables -A FORWARD -i "$FFT_IF" -o "$WAN_IF" -m comment --comment "$COMMENT" -j ACCEPT
iptables -A FORWARD -i "$WAN_IF" -o "$FFT_IF" -m state --state ESTABLISHED,RELATED -m comment --comment "$COMMENT" -j ACCEPT

echo "Current rules:"
iptables -t nat -L POSTROUTING -n -v
iptables -L FORWARD -n -v

echo
echo "=== Persist iptables rules ==="
export DEBIAN_FRONTEND=noninteractive
if ! dpkg -l 2>/dev/null | grep -q iptables-persistent; then
    echo iptables-persistent iptables-persistent/autosave_v4 boolean true | debconf-set-selections
    echo iptables-persistent iptables-persistent/autosave_v6 boolean true | debconf-set-selections
    apt-get update -qq
    apt-get install -y iptables-persistent
fi
netfilter-persistent save
echo "Saved to /etc/iptables/rules.v4"

echo
echo "=== This machine's FFT-side IP (gateway IP for drones) ==="
GW_IP=$(fft_interface_ip "$FFT_IF")
echo "$GW_IP"
echo
echo "Point drones at this machine with:"
echo "  ./pi_fft_gateway.sh <pi-host> <pi-user>"
echo "(run from this machine - it auto-detects $GW_IP as the gateway)"

echo
echo "=== DONE ==="
