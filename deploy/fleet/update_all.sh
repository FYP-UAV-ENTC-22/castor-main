#!/bin/bash
# Run castor-update on several Pis over SSH, one at a time, stopping at the
# first failure, then print which image revision each one is running.
#
#   deploy/fleet/update_all.sh [castor-update options] [user@host ...]
#
# Hosts come from the command line, or from deploy/fleet/hosts.txt (one
# user@host per line, # comments; gitignored, see hosts.example).
# Options are passed straight to castor-update: --check, --force, --tag T, --no-git.
#
# Every drone in a team must run the same build: the policy's observation layout
# and the topics between drones depend on it. For a flight day pin them all:
#   deploy/fleet/update_all.sh --tag sha-<12-char revision>
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
opts=()
hosts=()
while [ $# -gt 0 ]; do
    case "$1" in
        --tag) opts+=("$1" "$2"); shift 2 ;;
        --check|--force|--no-git) opts+=("$1"); shift ;;
        -h|--help) sed -n '2,15p' "$0" | sed 's/^# \?//'; exit 0 ;;
        -*) echo "update_all: unknown option '$1'" >&2; exit 2 ;;
        *) hosts+=("$1"); shift ;;
    esac
done
if [ ${#hosts[@]} -eq 0 ] && [ -f "$HERE/hosts.txt" ]; then
    mapfile -t hosts < <(grep -vE '^[[:space:]]*(#|$)' "$HERE/hosts.txt")
fi
[ ${#hosts[@]} -gt 0 ] || { echo "update_all: no hosts (pass user@host or create $HERE/hosts.txt)" >&2; exit 2; }

for h in "${hosts[@]}"; do
    printf '\n\033[1m######## %s\033[0m\n' "$h"
    ssh -o BatchMode=yes -o ConnectTimeout=8 "$h" castor-update "${opts[@]}"
done

printf '\n\033[1mRunning revisions\033[0m\n'
for h in "${hosts[@]}"; do
    rev=$(ssh -o BatchMode=yes -o ConnectTimeout=8 "$h" \
        "docker inspect --format '{{index .Config.Labels \"org.opencontainers.image.revision\"}}' castor-system-1" 2>/dev/null || echo "not running")
    printf '    %-32s %s\n' "$h" "$rev"
done
