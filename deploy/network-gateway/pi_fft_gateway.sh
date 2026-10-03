#!/bin/bash
# Configure a drone Pi's "FFT" NetworkManager connection to route internet
# traffic through a gateway machine running base_station_setup.sh. The Pis need
# this to pull CASTOR images when the field network has no internet.
#
# Usage: ./pi_fft_gateway.sh <pi-host-or-ip> <ssh-user> [gateway-ip|off]
# Examples:
#   ./pi_fft_gateway.sh drone2-pi.local drone2                 # point at me
#   ./pi_fft_gateway.sh drone2-pi.local drone2 192.168.1.42    # point at a teammate
#   ./pi_fft_gateway.sh drone2-pi.local drone2 off             # stop routing via any gateway
#
# If gateway-ip is omitted, it's auto-detected as THIS machine's IP on the
# FFT interface - i.e. running this script (with no 3rd arg) points the
# drone at you. Pass a gateway-ip explicitly to point a drone at a
# teammate's machine instead (e.g. whoever currently has internet). Pass
# "off" to remove the custom route/DNS entirely (back to FFT-only, no
# internet) - not required for safety (an unreachable gateway just fails
# closed on its own), but useful if you want the Pi to stop trying.
#
# Safe to re-run / re-point: clears any previously-set custom route on the
# Pi's FFT profile before adding the new one, so switching which teammate a
# drone routes through doesn't leave stale routes behind.
#
# Prereq on the Pi: already joined to FFT with a saved NetworkManager
# profile literally named "FFT" (check with `nmcli connection show` on the
# Pi if unsure), and your SSH key on it (../pi/add_my_key_to_pi.sh).
#
# Copied from the retired drone-ops repo. One change: no password argument.
# The old version put the Pi's password on the command line and piped it into
# `sudo -S`, which leaves it in shell history and process listings. This one
# opens an interactive session (ssh -t) and sudo asks for it on the Pi.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=deploy/network-gateway/lib/detect_fft.sh
source "$SCRIPT_DIR/lib/detect_fft.sh"

PI_HOST="${1:?usage: $0 <pi-host> <ssh-user> [gateway-ip|off]}"
PI_USER="${2:?usage: $0 <pi-host> <ssh-user> [gateway-ip|off]}"
GW="${3:-}"

ssh_pi() {
    ssh -t -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new "$PI_USER@$PI_HOST" "$1"
}

if [ "$GW" = "off" ]; then
    echo "Removing FFT gateway route/DNS from $PI_HOST (sudo on the Pi will ask for its password)"
    ssh_pi "
set -e
sudo nmcli connection modify FFT ipv4.routes ''
sudo nmcli connection modify FFT ipv4.dns '' ipv4.ignore-auto-dns no
sudo nmcli device reapply wlan0
sleep 1
echo '--- route table ---'
ip route
"
    exit 0
fi

if [ -z "$GW" ]; then
    FFT_IF=$(detect_fft_interface) || {
        echo "ERROR: this machine isn't connected to '$FFT_SSID' wifi, and no gateway-ip was given." >&2
        echo "Either connect to FFT, or pass the gateway IP as a 3rd argument:" >&2
        echo "  $0 $PI_HOST $PI_USER <gateway-ip>" >&2
        exit 1
    }
    GW=$(fft_interface_ip "$FFT_IF")
    echo "No gateway-ip given - auto-detected this machine's FFT address: $GW"
fi

CONN="FFT"
DNS="8.8.8.8"

echo "Pointing $PI_HOST's default route at $GW (sudo on the Pi will ask for its password)"

ssh_pi "
set -e
sudo nmcli connection modify '$CONN' ipv4.dns '$DNS' ipv4.ignore-auto-dns yes
sudo nmcli connection modify '$CONN' ipv4.routes ''
sudo nmcli connection modify '$CONN' +ipv4.routes '0.0.0.0/0 $GW 100'
sudo nmcli device reapply wlan0
sleep 1
echo '--- route table ---'
ip route
echo '--- ping gateway ($GW) over FFT ---'
ping -c 2 -W 2 $GW
echo '--- ping 8.8.8.8 ---'
ping -c 2 -W 2 8.8.8.8
echo '--- ping google.com (DNS check) ---'
ping -c 2 -W 2 google.com
echo '--- ghcr.io (image registry) ---'
curl -sS -o /dev/null -w 'ghcr.io HTTP %{http_code}\n' https://ghcr.io/v2/ || true
"
