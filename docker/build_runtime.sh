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
#   --push-by-digest DIR  CI: push untagged, write the digest to DIR/<component>-<arch>
#   --cache               read/write the registry build cache (<image>:buildcache-<arch>)
#   --test                run the component's colcon tests (component-test) first
#
# Examples:
#   docker/build_runtime.sh vehicle                       # load castor-vehicle:local
#   docker/build_runtime.sh --test                        # test and build all four
#   docker/build_runtime.sh --push --tag main system      # needs `docker login ghcr.io`
set -euo pipefail
# shellcheck source=docker/buildx_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/buildx_common.sh"

platform="$(castor_local_platform)"
tags=()
push=0
digest_dir=""
cache=0
test=0
components=()

while [ $# -gt 0 ]; do
    case "$1" in
        --platform) platform="$2"; shift 2 ;;
        --tag) tags+=("$2"); shift 2 ;;
        --push) push=1; shift ;;
        --push-by-digest) digest_dir="$2"; shift 2 ;;
        --cache) cache=1; shift ;;
        --test) test=1; shift ;;
        -h|--help) sed -n '2,22p' "$0" | sed 's/^# \?//'; exit 0 ;;
        -*) die "unknown option '$1' (try --help)" ;;
        *) castor_check_component "$1"; components+=("$1"); shift ;;
    esac
done
[ ${#components[@]} -gt 0 ] || components=("${CASTOR_COMPONENTS[@]}")
[ ${#tags[@]} -gt 0 ] || tags=(local)
if [ "$push" -eq 0 ] && [ -z "$digest_dir" ] && [[ "$platform" == *,* ]]; then
    die "a multi-platform build can't be loaded locally; add --push"
fi

revision="$(castor_revision)"
builder_args=()
if [ "$push" -eq 1 ] || [ -n "$digest_dir" ] || [ "$cache" -eq 1 ]; then
    builder_args=(--builder "$("$CASTOR_ROOT/docker/ensure_builder.sh")")
fi

for c in "${components[@]}"; do
    image="$(castor_image "$c")"
    castor_prepare_component "$c"
    common=(-f "$CASTOR_ROOT/docker/Dockerfile" --platform "$platform"
            --build-arg "COMPONENT=$c" --build-arg "CASTOR_REVISION=$revision" "${builder_args[@]}")

    if [ "$cache" -eq 1 ]; then
        arch="${platform//linux\//}"; arch="${arch//,/-}"; arch="${arch//\//}"
        common+=(--cache-from "type=registry,ref=$image:buildcache-$arch")
        if [ "$push" -eq 1 ] || [ -n "$digest_dir" ]; then
            common+=(--cache-to "type=registry,ref=$image:buildcache-$arch,mode=max")
        fi
    fi

    if [ "$test" -eq 1 ]; then
        say "Testing $c ($platform)"
        docker buildx build "${common[@]}" --target component-test --output type=cacheonly "$CASTOR_ROOT"
    fi

    say "Building $c-final ($platform, revision $revision)"
    if [ -n "$digest_dir" ]; then
        mkdir -p "$digest_dir"
        meta="$(mktemp)"
        docker buildx build "${common[@]}" --target "$c-final" \
            --output "type=image,name=$image,push-by-digest=true,name-canonical=true,push=true" \
            --metadata-file "$meta" "$CASTOR_ROOT"
        arch="${platform//linux\//}"; arch="${arch//\//}"
        python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["containerimage.digest"])' "$meta" \
            > "$digest_dir/$c-$arch"
        rm -f "$meta"
        echo "    digest $(cat "$digest_dir/$c-$arch")"
    else
        tag_args=()
        for t in "${tags[@]}"; do tag_args+=(-t "$image:$t"); done
        if [ "$push" -eq 1 ]; then out=(--push); else out=(--load); fi
        docker buildx build "${common[@]}" --target "$c-final" "${tag_args[@]}" "${out[@]}" "$CASTOR_ROOT"
        [ "$push" -eq 1 ] || docker image ls "$image:${tags[0]}" --format '    {{.Repository}}:{{.Tag}}  {{.Size}}'
    fi
done
