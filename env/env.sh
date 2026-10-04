#!/usr/bin/env bash
# Source this to get a working Isaac Lab / Isaac Sim Python shell.
#
#   source <workspace>/env/env.sh
#
# All it really does is `conda activate castor`. That env's activate.d hooks
# already do the important work:
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
# ROS 2: /opt/ros/humble is sourced from ~/.bashrc and is LEFT ON the path on
# purpose. isaacsim.ros2.bridge ships with ros_distro = "system_default", and
# bundles both humble/ and jazzy/ internal library sets. "system_default" means
# it prefers a sourced system ROS 2 and falls back to the bundled libs. Keeping
# Humble sourced is the supported configuration and is what lets the bridge see
# your own custom message packages.

CASTOR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CASTOR_ENV="${CASTOR_ENV:-castor}"

# shellcheck disable=SC1091
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate "$CASTOR_ENV" || {
    echo "env.sh: conda env '$CASTOR_ENV' not found, run ./setup.sh first" >&2
    return 1
}

export CASTOR_ROOT
export ISAACLAB_PATH="$CASTOR_ROOT/components/simulation/IsaacLab"
export MARL_EXT_PATH="$CASTOR_ROOT/components/planning/MARL_cooperative_aerial_manipulation_ext"
export PEGASUS_PATH="$CASTOR_ROOT/components/simulation/pegasus_simulator"

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

echo "CASTOR env ready:"
echo "  workspace  $CASTOR_ROOT"
echo "  python     $(command -v python)  ($(python --version 2>&1))"
echo "  isaac sim  $(readlink -f "$ISAACLAB_PATH/_isaac_sim")"
echo "  assets     $ISAAC_ASSET_ROOT"
echo "  ros 2      ${ROS_DISTRO:-<not sourced>} (bridge ros_distro=system_default)"
