#!/bin/bash
# Build the <component>-dev images for this machine, tagged castor-<component>:dev.
# They carry the build tools and every dependency; the repo itself is mounted
# at /home/ws by docker-compose.dev.yml. Dev images are never pushed.
#
#   docker/build_dev.sh [component...]     (default: all four)
set -euo pipefail
# shellcheck source=docker/buildx_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/buildx_common.sh"

components=("$@")
[ ${#components[@]} -gt 0 ] || components=("${CASTOR_COMPONENTS[@]}")

for c in "${components[@]}"; do
    castor_check_component "$c"
    castor_prepare_component "$c"
    say "Building castor-$c:dev"
    docker buildx build -f "$CASTOR_ROOT/docker/Dockerfile" --target "$c-dev" \
        --build-arg "COMPONENT=$c" \
        --build-arg "USER_UID=$(id -u)" --build-arg "USER_GID=$(id -g)" \
        -t "castor-$c:dev" --load "$CASTOR_ROOT"
done
