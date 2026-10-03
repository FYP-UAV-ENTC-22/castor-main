#!/bin/bash
# Create (once) the buildx builder that multi-platform pushes and registry
# caching need. Plain local builds use the default builder and don't need this.
set -euo pipefail
# shellcheck source=docker/buildx_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/buildx_common.sh"

if ! docker buildx inspect "$CASTOR_BUILDER" >/dev/null 2>&1; then
    say "Creating buildx builder '$CASTOR_BUILDER'"
    docker buildx create --name "$CASTOR_BUILDER" --driver docker-container --bootstrap >/dev/null
fi
echo "$CASTOR_BUILDER"
