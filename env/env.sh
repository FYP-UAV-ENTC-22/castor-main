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
export ISAACLAB_PATH="$CASTOR_ROOT/external/IsaacLab"
export MARL_EXT_PATH="$CASTOR_ROOT/external/MARL_cooperative_aerial_manipulation_ext"
export PEGASUS_PATH="$CASTOR_ROOT/external/pegasus_simulator"

echo "CASTOR env ready:"
echo "  workspace  $CASTOR_ROOT"
echo "  python     $(command -v python)  ($(python --version 2>&1))"
echo "  isaac sim  $(readlink -f "$ISAACLAB_PATH/_isaac_sim")"
echo "  ros 2      ${ROS_DISTRO:-<not sourced>} (bridge ros_distro=system_default)"
