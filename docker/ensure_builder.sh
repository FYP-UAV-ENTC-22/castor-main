#!/bin/bash
# Create (once) the docker-container buildx builder every onboard build uses:
# base images stay in its cache instead of the image store (no untagged images).
set -euo pipefail
# shellcheck source=docker/buildx_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/buildx_common.sh"

if ! docker buildx inspect "$CASTOR_BUILDER" >/dev/null 2>&1; then
    say "Creating buildx builder '$CASTOR_BUILDER'" >&2   # stdout is the builder name
    docker buildx create --name "$CASTOR_BUILDER" --driver docker-container --bootstrap >/dev/null
fi
echo "$CASTOR_BUILDER"
