#!/bin/bash
# Pull new CASTOR images and restart only the containers whose image changed.
# Never runs on its own: run it yourself, with `sudo systemctl start castor-update`,
# or for the whole fleet from a laptop with deploy/fleet/update_all.sh.
#
#   castor-update.sh [--check] [--force] [--tag TAG] [--no-git]
#
#   --check    show what would change; pulls nothing, restarts nothing
#   --force    skip the update gate (recovering a broken stack over SSH)
#   --tag TAG  deploy this image tag (e.g. sha-<rev> for a flight day). It is
#              remembered in /var/lib/castor/stack-tag.env, so the stack comes
#              back on the same tag after a reboot; --tag main returns to main.
#   --no-git   don't refresh the deploy files (compose file, these scripts)
#
# Refuses while the vehicle may be flying. The system container writes
# /run/castor/update_gate every second; only a fresh "allow" (IDLE and disarmed,
# or a bench Pi with fc.enabled: false) lets an update through. If no CASTOR
# component is running at all there is nothing to disrupt, so no gate is needed.
set -euo pipefail

SRC="${CASTOR_SRC:-/opt/castor/src}"
ENV_FILE=/etc/castor/stack.env
TAG_FILE=/var/lib/castor/stack-tag.env
GATE=/run/castor/update_gate
LOG=/var/log/castor/deployments.log
GATE_MAX_AGE_S=10
SERVICES=(vehicle localization planning system)
ZENOH_DEFAULT=ghcr.io/fyp-uav-entc-22/zenoh-bridge-ros2dds:1.10.1   # keep in step with docker-compose.prod.yml

die()  { printf '\033[31mcastor-update: %s\033[0m\n' "$*" >&2; exit 1; }
say()  { printf '\033[1m==> %s\033[0m\n' "$*"; }

# shellcheck disable=SC1090
[ -f "$ENV_FILE" ] && { set -a; source "$ENV_FILE"; set +a; }
# shellcheck disable=SC1090
[ -f "$TAG_FILE" ] && { set -a; source "$TAG_FILE"; set +a; }
CASTOR_REGISTRY="${CASTOR_REGISTRY:-ghcr.io/fyp-uav-entc-22}"
CASTOR_TAG="${CASTOR_TAG:-main}"

check=0
force=0
git_pull=1
while [ $# -gt 0 ]; do
    case "$1" in
        --check) check=1; shift ;;
        --force) force=1; shift ;;
        --tag) CASTOR_TAG="$2"; shift 2 ;;
        --no-git) git_pull=0; shift ;;
        -h|--help) sed -n '2,18p' "$0" | sed 's/^# \?//'; exit 0 ;;
        *) die "unknown option '$1' (try --help)" ;;
    esac
done
ZENOH_BRIDGE_IMAGE="${ZENOH_BRIDGE_IMAGE:-$ZENOH_DEFAULT}"
export CASTOR_REGISTRY CASTOR_TAG CASTOR_ROBOT_CONFIG ZENOH_BRIDGE_IMAGE
compose=(docker compose -f "$SRC/docker/docker-compose.prod.yml")
image() {
    if [ "$1" = zenoh-bridge ]; then echo "$ZENOH_BRIDGE_IMAGE"; else echo "$CASTOR_REGISTRY/castor-$1:$CASTOR_TAG"; fi
}
ALL=("${SERVICES[@]}" zenoh-bridge)

[ -f "$SRC/docker/docker-compose.prod.yml" ] || die "no checkout at $SRC (run deploy/pi/install.sh first)"
docker info >/dev/null 2>&1 || die "cannot talk to Docker (are you in the docker group, or root?)"

running_components() {
    docker ps --filter label=com.docker.compose.project=castor --filter status=running \
        --format '{{.Label "com.docker.compose.service"}}' | grep -xE 'vehicle|localization|planning|system' || true
}

gate_check() {
    if [ -z "$(running_components)" ]; then
        echo "    no CASTOR component is running; nothing to disrupt"
        return 0
    fi
    [ -f "$GATE" ] || die "no update gate at $GATE: the system container is not writing it.
    If you are certain the drone is on the ground and disarmed, re-run with --force."
    local verdict ts state age
    read -r verdict ts state < "$GATE" || true
    [[ "${ts:-}" =~ ^[0-9]+$ ]] || die "unreadable update gate: $(cat "$GATE")"
    age=$(( $(date +%s) - ts ))
    [ "$age" -le "$GATE_MAX_AGE_S" ] || die "update gate is ${age}s old (system container stalled?). Use --force if the drone is on the ground."
    [ "$verdict" = allow ] || die "update gate says '$verdict' in state ${state:-?}: the vehicle may be armed or mid-mission. Not updating."
    echo "    update gate: allow (state $state, ${age}s old)"
}

remote_digest() { docker buildx imagetools inspect "$1" --format '{{.Manifest.Digest}}' 2>/dev/null || true; }
local_digests() { docker image inspect --format '{{join .RepoDigests " "}}' "$1" 2>/dev/null || true; }

if [ "$check" -eq 1 ]; then
    say "Checking $CASTOR_REGISTRY/castor-*:$CASTOR_TAG and the zenoh bridge"
    for svc in "${ALL[@]}"; do
        img=$(image "$svc"); remote=$(remote_digest "$img"); have=$(local_digests "$img")
        if [ -z "$remote" ]; then status="not found in the registry"
        elif [ -z "$have" ]; then status="not pulled yet"
        elif [[ " $have " == *"@$remote "* ]]; then status="up to date"
        else status="UPDATE AVAILABLE"
        fi
        printf '    %-13s %s\n' "$svc" "$status"
    done
    exit 0
fi

say "Update gate"
if [ "$force" -eq 1 ]; then echo "    skipped (--force)"; else gate_check; fi

if [ "$git_pull" -eq 1 ]; then
    say "Deploy files ($SRC)"
    git -c safe.directory="$SRC" -C "$SRC" pull --ff-only --quiet || die "git pull failed in $SRC"
    echo "    at $(git -c safe.directory="$SRC" -C "$SRC" log -1 --format='%h %s')"
fi

declare -A before
for svc in "${ALL[@]}"; do
    before[$svc]=$(docker inspect --format '{{.Image}}' "castor-$svc-1" 2>/dev/null || true)
done

say "Pulling $CASTOR_TAG"
"${compose[@]}" pull --quiet \
    || die "pull failed; nothing was restarted. Check the network and that tag '$CASTOR_TAG' exists."

# A pull can take minutes over Wi-Fi; the vehicle may have been armed meanwhile.
if [ "$force" -eq 0 ]; then
    say "Update gate (again, before restarting)"
    gate_check
fi

say "Restarting changed containers"
"${compose[@]}" up -d --remove-orphans

mkdir -p "$(dirname "$LOG")"
deploy_rev=$(git -c safe.directory="$SRC" -C "$SRC" rev-parse --short=12 HEAD 2>/dev/null || echo unknown)
ros_restarted=0
for svc in "${ALL[@]}"; do
    img=$(image "$svc")
    digest=$(docker image inspect --format '{{index .RepoDigests 0}}' "$img" 2>/dev/null || echo "?")
    rev=$(docker image inspect --format '{{index .Config.Labels "org.opencontainers.image.revision"}}' "$img" 2>/dev/null || echo "?")
    now_id=$(docker inspect --format '{{.Image}}' "castor-$svc-1" 2>/dev/null || true)
    changed=$([ "${before[$svc]}" != "$now_id" ] && echo restarted || echo unchanged)
    [ "$changed" = restarted ] && [ "$svc" != zenoh-bridge ] && ros_restarted=1
    printf '    %-13s %-10s %s\n' "$svc" "$changed" "$rev"
    printf '%s %s tag=%s deploy=%s %s %s %s %s\n' "$(date -Is)" "$(hostname)" "$CASTOR_TAG" "$deploy_rev" \
        "$svc" "$changed" "$rev" "$digest" >> "$LOG"
done

# zenoh-plugin-ros2dds 1.10.1 (#722, fix in PR #738): when a restart removes a route, the route's matching
# listener lives on and re-creates its DDS reader at the next ground-station (re)connect, so from then on
# every sample of that topic crosses twice. Restarting the bridge drops those listeners; it costs a few
# seconds of cross-host traffic. Drop this once the bridge image carries the fix.
if [ "$ros_restarted" -eq 1 ] && [ "${before[zenoh-bridge]}" = "$(docker inspect --format '{{.Image}}' castor-zenoh-bridge-1 2>/dev/null)" ]; then
    "${compose[@]}" restart zenoh-bridge >/dev/null 2>&1 \
        && echo "    restarted zenoh-bridge to drop stale routes (zenoh-plugin-ros2dds #722)"
fi

# Remember the tag, so castor-stack.service starts the same images after a reboot.
printf 'CASTOR_TAG=%s\n' "$CASTOR_TAG" > "$TAG_FILE"

# Remove the images this update replaced, by ID: never a blanket prune.
for svc in "${ALL[@]}"; do
    old=${before[$svc]}
    now=$(docker inspect --format '{{.Image}}' "castor-$svc-1" 2>/dev/null || true)
    if [ -n "$old" ] && [ "$old" != "$now" ] && [ -z "$(docker ps -aq --filter "ancestor=$old")" ]; then
        docker image rm "$old" >/dev/null 2>&1 && echo "    removed old $svc image ${old:7:12}" || true
    fi
done
echo "    logged to $LOG"
