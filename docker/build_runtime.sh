#!/bin/bash
# Build the runtime (<component>-final) images. Used locally and by CI.
#
#   docker/build_runtime.sh [options] [component...]     (default: all four)
#
#   --platform P          linux/amd64, linux/arm64, or both comma-separated.
#                         Default: this machine's. Another architecture needs
#                         QEMU (docker run --privileged --rm tonistiigi/binfmt --install arm64)
#                         or a native machine; CI builds arm64 on arm64 runners.
#   --tag T               tag to apply (repeatable). Default: local
#   --push                push the tags to $CASTOR_REGISTRY instead of loading locally
#   --push-arch TAG       CI: push one platform as <image>:TAG-<arch> (a tagged
#                         per-arch image that the publish job joins into one index)
#   --cache               CI: read/write the GitHub Actions build cache
#   --test                run the component's colcon tests (component-test) first
#
# Every build goes through the docker-container builder ($CASTOR_BUILDER), so
# base images stay in BuildKit's cache and never show up as untagged images, and
# nothing carries provenance/SBOM attestations (those are untagged manifests on
# GHCR). A local rebuild removes the image it replaced, by ID.
#
# Examples:
#   docker/build_runtime.sh vehicle                       # load ghcr.io/.../castor-vehicle:local
#   docker/build_runtime.sh --test                        # test and build all four
#   docker/build_runtime.sh --push --tag main system      # needs `docker login ghcr.io`
set -euo pipefail
# shellcheck source=docker/buildx_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/buildx_common.sh"

platform="$(castor_local_platform)"
tags=()
push=0
arch_tag=""
cache=0
test=0
components=()

while [ $# -gt 0 ]; do
    case "$1" in
        --platform) platform="$2"; shift 2 ;;
        --tag) tags+=("$2"); shift 2 ;;
        --push) push=1; shift ;;
        --push-arch) arch_tag="$2"; shift 2 ;;
        --cache) cache=1; shift ;;
        --test) test=1; shift ;;
        -h|--help) sed -n '2,26p' "$0" | sed 's/^# \?//'; exit 0 ;;
        -*) die "unknown option '$1' (try --help)" ;;
        *) castor_check_component "$1"; components+=("$1"); shift ;;
    esac
done
[ ${#components[@]} -gt 0 ] || components=("${CASTOR_COMPONENTS[@]}")
[ ${#tags[@]} -gt 0 ] || tags=(local)
if [ "$push" -eq 0 ] && [ -z "$arch_tag" ] && [[ "$platform" == *,* ]]; then
    die "a multi-platform build can't be loaded locally; add --push"
fi
if [ -n "$arch_tag" ] && [[ "$platform" == *,* ]]; then
    die "--push-arch takes a single platform"
fi

revision="$(castor_revision)"
builder="$("$CASTOR_ROOT/docker/ensure_builder.sh")"
arch="${platform//linux\//}"; arch="${arch//,/-}"; arch="${arch//\//}"

for c in "${components[@]}"; do
    image="$(castor_image "$c")"
    castor_prepare_component "$c"
    common=(-f "$CASTOR_ROOT/docker/Dockerfile" --platform "$platform" --builder "$builder"
            --provenance=false --sbom=false
            --build-arg "COMPONENT=$c" --build-arg "CASTOR_REVISION=$revision")

    if [ "$cache" -eq 1 ]; then
        common+=(--cache-from "type=gha,scope=$c-$arch" --cache-to "type=gha,scope=$c-$arch,mode=max")
    fi

    if [ "$test" -eq 1 ]; then
        say "Testing $c ($platform)"
        docker buildx build "${common[@]}" --target component-test --output type=cacheonly "$CASTOR_ROOT"
    fi

    say "Building $c-final ($platform, revision $revision)"
    if [ -n "$arch_tag" ]; then
        docker buildx build "${common[@]}" --target "$c-final" -t "$image:$arch_tag-$arch" --push "$CASTOR_ROOT"
    elif [ "$push" -eq 1 ]; then
        tag_args=()
        for t in "${tags[@]}"; do tag_args+=(-t "$image:$t"); done
        docker buildx build "${common[@]}" --target "$c-final" "${tag_args[@]}" --push "$CASTOR_ROOT"
    else
        old="$(docker image inspect --format '{{.Id}}' "$image:${tags[0]}" 2>/dev/null || true)"
        tag_args=()
        for t in "${tags[@]}"; do tag_args+=(-t "$image:$t"); done
        docker buildx build "${common[@]}" --target "$c-final" "${tag_args[@]}" --load "$CASTOR_ROOT"
        castor_remove_if_dangling "$old"
        docker image ls "$image:${tags[0]}" --format '    {{.Repository}}:{{.Tag}}  {{.Size}}'
    fi
done
