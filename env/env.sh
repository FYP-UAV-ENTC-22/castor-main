#!/usr/bin/env bash
# Source this to get an Isaac Sim / Isaac Lab Python for the workspace.
#
#   source <workspace>/env/env.sh
#   isaac-python <script.py> [args]     # run anything that needs Isaac, from anywhere in the repo
#   isaac-shell                         # a shell next to Isaac (container mode)
#
# Which Isaac, in order (CASTOR_ISAAC=container|native forces one):
#   1. container  the simulation image (make sim-image or make sim-pull). The container
#                 is started here (make sim-up); isaac-python runs
#                 /isaac-sim/python.sh in it, in the same directory of the repo
#                 (mounted at /home/ws), then gives root-owned output back
#                 (make sim-own). No external drive needed: NVIDIA's asset pack is
#                 optional there (docker/README.md, "Simulation image").
#   2. native     the Isaac Sim install that IsaacLab/_isaac_sim points at (the
#                 external SSD, /mnt/isaac/isaacsim), through the `castor` conda
#                 env; isaac-python is that env's python. Details below.
#   3. neither    an error, and nothing is changed.
#
# Native mode is `conda activate castor`. That env's activate.d hooks do the work:
#   * etc/conda/activate.d/setenv.sh   - sources _isaac_sim/setup_conda_env.sh,
#     which puts the Kit directories (libcarb.so, plugins, bindings) on
#     LD_LIBRARY_PATH and sets CARB_APP_PATH / EXP_PATH / ISAAC_PATH.
#   * etc/conda/activate.d/torch_gomp.sh - LD_PRELOADs the torch-bundled
#     libgomp.so.1, avoiding the OpenMP double-load crash.
# Both are written by `isaaclab.sh --conda`, which setup.sh runs for you.
#
# VERIFIED on 2026-08-21 (as the `isaaclab` env, before the CASTOR rename):
# headless MAPPO training on Isaac-flycrane-payload-decentralized-hovering-v0
# runs clean at ~23 it/s.
#
# Do NOT do `env -u LD_LIBRARY_PATH python train.py`. That trick is only for the
# GUI launcher (isaac-sim.sh sets its own paths internally). For the conda
# Python it removes the Kit paths too, and you get:
#     libcarb.so: cannot open shared object file
#     TypeError: 'NoneType' object is not callable   (SimulationApp is None)
#
# ROS 2 (native): /opt/ros/humble is sourced from ~/.bashrc and is LEFT ON the
# path on purpose. isaacsim.ros2.bridge ships with ros_distro = "system_default",
# and bundles both humble/ and jazzy/ internal library sets. "system_default"
# means it prefers a sourced system ROS 2 and falls back to the bundled libs.
# Keeping Humble sourced is the supported configuration and is what lets the
# bridge see your own custom message packages. The container uses its bundled
# Jazzy (docker/README.md).

CASTOR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CASTOR_ENV="${CASTOR_ENV:-castor}"
CASTOR_SIM_IMAGE="${CASTOR_SIM_IMAGE:-ghcr.io/fyp-uav-entc-22/castor-simulation:latest}"
export CASTOR_ROOT
export ISAACLAB_PATH="$CASTOR_ROOT/components/simulation/IsaacLab"
export MARL_EXT_PATH="$CASTOR_ROOT/components/planning/MARL_cooperative_aerial_manipulation_ext"
export PEGASUS_PATH="$CASTOR_ROOT/components/simulation/pegasus_simulator"

_castor_have_container() {
    command -v docker >/dev/null 2>&1 && docker image inspect "$CASTOR_SIM_IMAGE" >/dev/null 2>&1
}
# The native install lives on an external drive; a missing drive fails this test quickly (fstab device timeout).
_castor_have_native() {
    [ -e "$ISAACLAB_PATH/_isaac_sim/python.sh" ] && command -v conda >/dev/null 2>&1
}

case "${CASTOR_ISAAC:-auto}" in
    auto)
        if _castor_have_container; then CASTOR_ISAAC=container
        elif _castor_have_native; then CASTOR_ISAAC=native
        else CASTOR_ISAAC=none
        fi ;;
    container) _castor_have_container || CASTOR_ISAAC=none ;;
    native) _castor_have_native || CASTOR_ISAAC=none ;;
    *) echo "env.sh: CASTOR_ISAAC must be container, native or unset, not '$CASTOR_ISAAC'" >&2; return 1 ;;
esac

if [ "$CASTOR_ISAAC" = none ]; then
    echo "env.sh: no Isaac Sim found. Either" >&2
    echo "  - build the simulation image ($CASTOR_SIM_IMAGE): make sim-image   (docker/README.md), or" >&2
    echo "  - mount the native install that $ISAACLAB_PATH/_isaac_sim points at ($(readlink "$ISAACLAB_PATH/_isaac_sim" 2>/dev/null || echo 'no link'))" >&2
    echo "    and run ./setup.sh for the '$CASTOR_ENV' conda env" >&2
    unset CASTOR_ISAAC
    return 1
fi
export CASTOR_ISAAC

if [ "$CASTOR_ISAAC" = container ]; then
    make -s --no-print-directory -C "$CASTOR_ROOT" sim-up || {
        echo "env.sh: could not start the simulation container (make sim-up)" >&2
        return 1
    }

    # The container sees only the repo, at /home/ws: map the working directory and any argument under the repo.
    _castor_sim_exec() {
        local rel="${PWD#"$CASTOR_ROOT"}" a args=()
        if [ "$rel" = "$PWD" ]; then
            echo "isaac: cd into $CASTOR_ROOT first (the container sees only the repo)" >&2
            return 2
        fi
        for a in "$@"; do args+=("${a//"$CASTOR_ROOT"//home/ws}"); done
        if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then xhost +local: >/dev/null 2>&1; fi
        docker compose -f "$CASTOR_ROOT/docker/docker-compose.sim.yml" exec -w "/home/ws$rel" \
            -e DISPLAY="${DISPLAY:-}" simulation "${args[@]}"
        local s=$?
        make -s --no-print-directory -C "$CASTOR_ROOT" sim-own
        return $s
    }
    isaac-python() { _castor_sim_exec /isaac-sim/python.sh "$@"; }
    isaac-shell() { _castor_sim_exec bash "$@"; }

    echo "CASTOR env ready (Isaac in the container):"
    echo "  workspace  $CASTOR_ROOT  (/home/ws in the container)"
    echo "  isaac sim  $CASTOR_SIM_IMAGE, container $(docker compose -f "$CASTOR_ROOT/docker/docker-compose.sim.yml" ps -q simulation | cut -c1-12)"
    echo "  python     isaac-python  (/isaac-sim/python.sh)"
    echo "  assets     CASTOR's from the repo; NVIDIA's from a local pack if mounted, else S3"
    echo "  ros 2      bundled Jazzy, domain ${ROS_DOMAIN_ID:-20} (docker/README.md)"
    return 0
fi

# native
# shellcheck disable=SC1091
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate "$CASTOR_ENV" || {
    echo "env.sh: conda env '$CASTOR_ENV' not found, run ./setup.sh first" >&2
    return 1
}
isaac-python() { python "$@"; }
isaac-shell() { echo "isaac-shell: native mode, this shell is already set up" >&2; }

# Asset root: Isaac Lab's .kit files default it to NVIDIA's S3 bucket, and a
# persistent value in Kit's per-app user.config.json overrides the .kit file.
# Point that value at a local asset pack to load offline. It is not settable
# from the environment: OMNI_KIT_* maps every "_" to "/", and asset_root has one.
ISAAC_ASSET_ROOT="$(python - "$ISAACLAB_PATH/_isaac_sim/kit/data/Kit/Isaac-Sim/5.1/user.config.json" <<'EOF' 2>/dev/null
import json, sys
print(json.load(open(sys.argv[1]))["persistent"]["isaac"]["asset_root"]["cloud"])
EOF
)"
ISAAC_ASSET_ROOT="${ISAAC_ASSET_ROOT:-<.kit default, S3>}"

echo "CASTOR env ready (native Isaac):"
echo "  workspace  $CASTOR_ROOT"
echo "  python     $(command -v python)  ($(python --version 2>&1)), also isaac-python"
echo "  isaac sim  $(readlink -f "$ISAACLAB_PATH/_isaac_sim")"
echo "  assets     $ISAAC_ASSET_ROOT"
echo "  ros 2      ${ROS_DISTRO:-<not sourced>} (bridge ros_distro=system_default)"
