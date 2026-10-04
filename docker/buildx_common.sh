# Shared settings for the build scripts. Source it; don't run it.
# shellcheck shell=bash

CASTOR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Images are ghcr.io/<org>/castor-<component>. GHCR scopes images to the GitHub
# account that owns them, so the namespace is the org; override for a fork.
CASTOR_REGISTRY="${CASTOR_REGISTRY:-ghcr.io/fyp-uav-entc-22}"
CASTOR_COMPONENTS=(vehicle localization planning system)
CASTOR_BUILDER="${CASTOR_BUILDER:-castor}"

die()  { printf '\033[31merror: %s\033[0m\n' "$*" >&2; exit 1; }
say()  { printf '\n\033[1m==> %s\033[0m\n' "$*"; }

castor_image() { echo "$CASTOR_REGISTRY/castor-$1"; }

# Git revision baked into every image (and published in each heartbeat).
castor_revision() {
    local rev
    rev=$(git -C "$CASTOR_ROOT" rev-parse --short=12 HEAD 2>/dev/null || echo unknown)
    # Only what goes into an image counts (not submodule pointers or components/simulation).
    if ! git -C "$CASTOR_ROOT" diff --quiet --ignore-submodules=all HEAD -- \
            components/common components/vehicle components/localization components/planning \
            components/system docker .devcontainer 2>/dev/null; then
        rev="$rev-dirty"
    fi
    echo "$rev"
}

# After a rebuild re-tags an image, remove the one it replaced if nothing else
# refers to it. By ID only: never a blanket prune on a shared machine.
castor_remove_if_dangling() {
    local old="$1"
    [ -n "$old" ] || return 0
    [ -z "$(docker image inspect --format '{{join .RepoTags " "}}' "$old" 2>/dev/null)" ] || return 0
    [ -z "$(docker ps -aq --filter "ancestor=$old")" ] || return 0
    docker image rm "$old" >/dev/null 2>&1 && echo "    removed the replaced image ${old:7:12}" || true
}

castor_check_component() {
    local c
    for c in "${CASTOR_COMPONENTS[@]}"; do [ "$c" = "$1" ] && return 0; done
    die "unknown component '$1' (expected: ${CASTOR_COMPONENTS[*]})"
}

# Generated inputs a component needs before docker sees its context.
castor_prepare_component() {
    case "$1" in
        vehicle) "$CASTOR_ROOT/docker/sync_px4_msgs.sh" ;;
    esac
}

castor_local_platform() {
    case "$(uname -m)" in
        x86_64) echo linux/amd64 ;;
        aarch64|arm64) echo linux/arm64 ;;
        *) die "unsupported host architecture $(uname -m)" ;;
    esac
}
