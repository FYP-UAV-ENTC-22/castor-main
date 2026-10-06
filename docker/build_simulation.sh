#!/bin/bash
# Build the simulation image, ghcr.io/fyp-uav-entc-22/castor-simulation:latest (Isaac Sim 5.1, Isaac Lab,
# Pegasus, PX4 SITL toolchain, ROS 2 Jazzy, skrl + the MARL ext). One name only: the build replaces it, the image
# it replaced is removed, `make sim-push` publishes it to the private GHCR package and `make sim-pull` fetches it.
# Never push it anywhere public: NVIDIA's licence forbids redistributing Isaac Sim.
#
#   docker/build_simulation.sh
#
# The base is pulled by tag and must match SIM_BASE_DIGEST. The build uses
# Docker's default builder (not the castor docker-container builder) so the
# ~25 GB of Isaac layers are stored once, in the image store. Only package
# metadata is sent as build context: code comes from the repo mounted at run time.
set -euo pipefail
# shellcheck source=docker/buildx_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/buildx_common.sh"

SIM_BASE_IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0
SIM_BASE_DIGEST=sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54
IMAGE="${CASTOR_SIM_IMAGE:-$(castor_image simulation):latest}"
STAGE="$CASTOR_ROOT/.docker/sim-meta"

say "Base image $SIM_BASE_IMAGE"
if ! docker image inspect "$SIM_BASE_IMAGE" >/dev/null 2>&1; then
    docker pull "$SIM_BASE_IMAGE"
fi
docker image inspect --format '{{join .RepoDigests " "}}' "$SIM_BASE_IMAGE" | grep -q "@$SIM_BASE_DIGEST" \
    || die "$SIM_BASE_IMAGE is not $SIM_BASE_DIGEST (NVIDIA re-tagged it?). Check, then update SIM_BASE_DIGEST."
echo "    digest $SIM_BASE_DIGEST"

# Two groups, one image layer each: isaaclab/ (rarely changes) and castor/ (skrl and the MARL ext).
say "Staging package metadata ($STAGE)"
rm -rf "$STAGE"
mkdir -p "$STAGE/isaaclab/source" "$STAGE/castor"
C="$CASTOR_ROOT/components"
# stub <dest> <module>: the package directory setup.py expects, empty
stub() { mkdir -p "$1/$2"; : > "$1/$2/__init__.py"; }
for p in isaaclab isaaclab_assets isaaclab_tasks isaaclab_rl isaaclab_mimic; do
    src="$C/simulation/IsaacLab/source/$p"; dst="$STAGE/isaaclab/source/$p"
    mkdir -p "$dst/config"
    cp "$src/setup.py" "$src/config/extension.toml" "$dst/"
    mv "$dst/extension.toml" "$dst/config/"
    [ -f "$src/pyproject.toml" ] && cp "$src/pyproject.toml" "$dst/"
    stub "$dst" "$p"
done
dst="$STAGE/castor/skrl"; mkdir -p "$dst"
cp "$C/planning/skrl/setup.py" "$C/planning/skrl/pyproject.toml" "$C/planning/skrl/README.md" "$dst/"
stub "$dst" skrl
src="$C/planning/MARL_cooperative_aerial_manipulation_ext/exts/MARL_mav_carry_ext"; dst="$STAGE/castor/MARL_mav_carry_ext"
mkdir -p "$dst/config"
cp "$src/setup.py" "$dst/"; cp "$src/config/extension.toml" "$dst/config/"
[ -f "$src/pyproject.toml" ] && cp "$src/pyproject.toml" "$dst/"
stub "$dst" MARL_mav_carry_ext
cp "$C/vehicle/PX4-Autopilot/Tools/setup/requirements.txt" "$STAGE/px4-requirements.txt"
du -sh "$STAGE" | sed 's/^/    /'

say "Building $IMAGE"
old="$(docker image inspect --format '{{.Id}}' "$IMAGE" 2>/dev/null || true)"
docker buildx build --builder default -f "$CASTOR_ROOT/docker/Dockerfile" --target simulation \
    --build-arg "SIM_BASE_IMAGE=$SIM_BASE_IMAGE" \
    --build-context "simmeta=$STAGE" \
    --label "org.opencontainers.image.revision=$(castor_revision)" \
    -t "$IMAGE" --load "$CASTOR_ROOT"
castor_remove_if_dangling "$old"
docker image ls "$IMAGE" --format '    {{.Repository}}:{{.Tag}}  {{.Size}}'
