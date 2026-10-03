#!/bin/bash
# Copy a Pi's CASTOR logs back to this machine: MCAP recordings (system
# record:=true), and deployments.log (which images ran when).
#
# Usage: deploy/pi/fetch_logs.sh <pi-host> <pi-user> [local-dest]
#        local-dest defaults to ./castor-logs/<pi-host>/
#
# Additive and safe to re-run: rsync without --delete, so nothing on the Pi is
# removed and logs already fetched are left alone. Adapted from drone-ops.
set -euo pipefail

PI_HOST="${1:?usage: $0 <pi-host> <pi-user> [local-dest]}"
PI_USER="${2:?usage: $0 <pi-host> <pi-user> [local-dest]}"
DEST="${3:-castor-logs/$PI_HOST}"
REMOTE=/var/log/castor

command -v rsync >/dev/null || { echo "rsync is not installed here (sudo apt-get install -y rsync)" >&2; exit 1; }

if ! ssh -o BatchMode=yes -o ConnectTimeout=8 "$PI_USER@$PI_HOST" "[ -d $REMOTE ]"; then
    echo "No $REMOTE on $PI_HOST yet; nothing to fetch."
    exit 0
fi

mkdir -p "$DEST"
echo "Fetching $PI_USER@$PI_HOST:$REMOTE/ -> $DEST/"
rsync -az --itemize-changes -e "ssh -o BatchMode=yes -o ConnectTimeout=8" \
    "$PI_USER@$PI_HOST:$REMOTE/" "$DEST/"
echo "Logs are in $DEST"
