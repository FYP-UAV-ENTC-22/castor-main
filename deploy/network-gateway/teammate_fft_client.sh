#!/bin/bash
# Lets a teammate's OWN laptop borrow another teammate's FFT gateway as a
# FALLBACK internet path: if this laptop has its own real internet, that
# always wins; FFT is only used when this laptop has no internet of its own.
# Never demotes or breaks an existing working connection.
#
# Usage (run locally, with sudo, on the teammate's own laptop - NOT over
# SSH against someone else's machine):
#   sudo ./teammate_fft_client.sh on <gateway-ip>   # enable FFT fallback internet
#   sudo ./teammate_fft_client.sh off               # disable, revert to normal
#
# <gateway-ip> is whichever teammate is currently running base_station_setup.sh
# (their FFT-side IP, printed at the end of that script).
#
# Safety note: NetworkManager does NOT necessarily use the route metric you
# ask for literally - it can add its own automatic priority offset on top
# (observed: asking for metric 150 became kernel metric 20150 on one
# machine). So this script doesn't trust the number it requests - after
# applying, it re-reads the ACTUAL kernel route table and verifies the
# fallback route really does lose to this machine's own internet. If that
# verification fails for any reason, it automatically rolls back instead of
# leaving a route active that might have silently demoted real internet.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/detect_fft.sh"

clear_fft_route_dns() {
    local conn="$1"
    nmcli connection modify "$conn" ipv4.routes ''
    nmcli connection modify "$conn" ipv4.dns '' ipv4.ignore-auto-dns no
    nmcli device reapply "$FFT_IF" >/dev/null 2>&1 || true
}

ACTION="${1:-}"

if [ "$ACTION" = "off" ]; then
    FFT_IF=$(detect_fft_interface) || { echo "Not connected to FFT right now - nothing to turn off."; exit 0; }
    CONN=$(nmcli -g GENERAL.CONNECTION device show "$FFT_IF")
    echo "Removing FFT fallback route/DNS from '$CONN'..."
    clear_fft_route_dns "$CONN"
    echo "Done - FFT is back to providing no internet (as normal)."
    exit 0
fi

GW="${2:-}"
if [ "$ACTION" != "on" ] || [ -z "$GW" ]; then
    echo "Usage: $0 on <gateway-ip>   |   $0 off" >&2
    exit 1
fi

FFT_IF=$(detect_fft_interface) || {
    echo "ERROR: not connected to '$FFT_SSID' wifi." >&2
    exit 1
}
CONN=$(nmcli -g GENERAL.CONNECTION device show "$FFT_IF")
echo "FFT interface: $FFT_IF (connection: $CONN)"

# Best-effort metric to request - not trusted literally, see safety note above.
best_own_metric_now() {
    ip -o route show default | awk -v f="$FFT_IF" '$5!=f' | grep -oP 'metric \K[0-9]+' | sort -n | head -1
}
own_metric_before=$(best_own_metric_now || true)
if [ -n "$own_metric_before" ]; then
    request_metric=$((own_metric_before + 50))
else
    request_metric=100
fi

echo "Requesting fallback route metric: $request_metric (will verify actual result below)"
nmcli connection modify "$CONN" ipv4.dns "8.8.8.8" ipv4.ignore-auto-dns yes
nmcli connection modify "$CONN" ipv4.routes ''
nmcli connection modify "$CONN" +ipv4.routes "0.0.0.0/0 $GW $request_metric"
nmcli device reapply "$FFT_IF"
sleep 1

echo
echo "--- resulting default routes ---"
ip route show default

# Verify: our fallback route's ACTUAL kernel metric must be strictly worse
# than the best of this machine's own non-FFT default routes (if any).
fallback_metric=$(ip -o route show default | awk -v f="$FFT_IF" -v gw="$GW" '$5==f && $3==gw' | grep -oP 'metric \K[0-9]+' | head -1)
own_metric_after=$(best_own_metric_now || true)

fail_reason=""
if [ -z "$fallback_metric" ]; then
    fail_reason="couldn't find the fallback route in the kernel table after applying it"
elif [ -n "$own_metric_after" ] && [ "$fallback_metric" -le "$own_metric_after" ]; then
    fail_reason="fallback route metric ($fallback_metric) is NOT worse than your own internet's metric ($own_metric_after) - it would have taken priority over your real connection"
fi

if [ -n "$fail_reason" ]; then
    echo
    echo "SAFETY CHECK FAILED: $fail_reason" >&2
    echo "Rolling back automatically - your connection has NOT been changed." >&2
    clear_fft_route_dns "$CONN"
    exit 1
fi

echo
echo "Verified safe: fallback metric $fallback_metric loses to your own internet (metric ${own_metric_after:-none})."
echo "Disable any time with: $0 off"
