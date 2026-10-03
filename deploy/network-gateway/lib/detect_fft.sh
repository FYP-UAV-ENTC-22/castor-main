#!/bin/bash
# Shared detection helpers for the FFT drone gateway scripts.
# Source this file (do not execute it directly).
#
# Designed for a team where multiple laptops may be connected to FFT at
# once, but internet access moves around (whoever currently has a dongle/
# hotspot plugged in). Every function here fails loudly instead of guessing,
# so a machine with no internet right now doesn't get half-configured.

FFT_SSID="${FFT_SSID:-FFT}"

# Prints the network interface currently connected to the FFT SSID.
# Returns non-zero and prints nothing if this machine isn't on FFT.
detect_fft_interface() {
    local dev conn ssid
    while IFS=: read -r dev conn; do
        [ -z "$conn" ] && continue
        ssid=$(nmcli -g 802-11-wireless.ssid connection show "$conn" 2>/dev/null)
        if [ "$ssid" = "$FFT_SSID" ]; then
            echo "$dev"
            return 0
        fi
    done < <(nmcli -t -f DEVICE,CONNECTION,TYPE dev status | awk -F: '$3=="wifi"{print $1":"$2}')
    return 1
}

# Prints this machine's IPv4 address on the given interface.
fft_interface_ip() {
    ip -4 -o addr show "$1" | awk '{print $4}' | cut -d/ -f1 | head -n1
}

# Tests real internet connectivity out a specific interface. Checking for
# "a default route" isn't enough - FFT's own dead-end DHCP gateway has one
# of those too; this actually sends a packet and expects a reply.
interface_has_internet() {
    local ifc="$1"
    ping -I "$ifc" -c 1 -W 2 1.1.1.1 >/dev/null 2>&1 && return 0
    ping -I "$ifc" -c 1 -W 2 8.8.8.8 >/dev/null 2>&1 && return 0
    return 1
}

# Prints the first non-FFT, non-virtual interface that has a default route
# AND verified internet connectivity. Non-zero exit and nothing printed if
# no such interface exists right now (i.e. this machine has no internet).
detect_internet_interface() {
    local exclude="$1" ifc
    for ifc in $(ip -o route show default | awk '{print $5}' | sort -u); do
        [ "$ifc" = "$exclude" ] && continue
        case "$ifc" in
            docker*|br-*|veth*|virbr*|tun*|tailscale*|wg*|lo) continue ;;
        esac
        if interface_has_internet "$ifc"; then
            echo "$ifc"
            return 0
        fi
    done
    return 1
}
