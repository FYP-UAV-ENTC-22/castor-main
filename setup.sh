#!/usr/bin/env bash
#
# CASTOR workspace setup.
#
#   ./setup.sh                 full setup: submodules, conda env, developer docs
#   ./setup.sh --recursive     also fetch nested submodules (PX4 pulls ~35, several GB)
#   ./setup.sh --skip-env      submodules and docs only (fast, for re-runs)
#   ./setup.sh --skip-docs     skip the developer-docs step
#   ./setup.sh --help
#
# Isaac Sim is NOT installed by this script. It is a ~10 GB binary install that
# lives outside the workspace. Point ISAACSIM_PATH at it (see below) or export
# ISAACSIM_PATH before running.

set -euo pipefail

# ---------------------------------------------------------------- configuration

# Where Isaac Sim 5.1 is installed. On the group's workstation this is an
# external SSD. Override by exporting ISAACSIM_PATH before running.
ISAACSIM_PATH="${ISAACSIM_PATH:-/mnt/isaac/isaacsim}"

# Name of the conda environment this script creates.
CONDA_ENV="${CONDA_ENV:-castor}"

# Private developer-docs repo. Cloning it requires org membership; the script
# skips it cleanly for everyone else.
DEV_DOCS_REPO="git@github.com:FYP-UAV-ENTC-22/castor-dev-docs.git"

# --------------------------------------------------------------------- plumbing

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACLAB_DIR="$ROOT/external/IsaacLab"

SKIP_ENV=0
SKIP_DOCS=0
RECURSIVE=0

while [ $# -gt 0 ]; do
    case "$1" in
        --recursive) RECURSIVE=1 ;;
        --skip-env)  SKIP_ENV=1 ;;
        --skip-docs) SKIP_DOCS=1 ;;
        --help|-h)   sed -n '3,13p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        *)           echo "setup.sh: unknown option '$1' (try --help)" >&2; exit 2 ;;
    esac
    shift
done

say()  { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[33m    %s\033[0m\n' "$*"; }
die()  { printf '\033[31merror: %s\033[0m\n' "$*" >&2; exit 1; }

# ------------------------------------------------------- 1. submodules

# Top level only by default. PX4-Autopilot alone carries ~35 nested submodules
# and several GB that nobody working on the RL side needs; `make px4_sitl` in
# that repo fetches them on demand anyway.
say "Fetching submodules"
if [ "$RECURSIVE" -eq 1 ]; then
    git -C "$ROOT" submodule update --init --recursive
else
    git -C "$ROOT" submodule update --init
fi
warn "$(git -C "$ROOT" submodule status | wc -l) submodules checked out"
[ "$RECURSIVE" -eq 1 ] || warn "nested submodules skipped; pass --recursive if you need PX4 firmware builds"

# ------------------------------------------------------- 2. Isaac Sim link

say "Linking Isaac Sim"
[ -d "$ISAACSIM_PATH" ] || die "Isaac Sim not found at $ISAACSIM_PATH
    Install it, mount the drive it lives on, or re-run with:
        ISAACSIM_PATH=/path/to/isaacsim ./setup.sh"

ln -sfn "$ISAACSIM_PATH" "$ISAACLAB_DIR/_isaac_sim"
warn "external/IsaacLab/_isaac_sim -> $ISAACSIM_PATH"

# ------------------------------------------------------- 3. conda environment

if [ "$SKIP_ENV" -eq 1 ]; then
    say "Skipping conda environment (--skip-env)"
else
    say "Building conda environment '$CONDA_ENV'"
    command -v conda >/dev/null || die "conda not found on PATH. Install Miniconda or Anaconda first."

    # shellcheck disable=SC1091
    # conda's activation machinery, and the Isaac Sim setup_conda_env.sh that
    # the activate.d hook sources, both read unset variables (ZSH_VERSION among
    # them). Under `set -u` that aborts the script right after the env is
    # created and before anything is installed into it. Drop nounset here.
    set +u
    source "$(conda info --base)/etc/profile.d/conda.sh"

    # isaaclab.sh --conda creates the env from environment.yml (python 3.11 for
    # Isaac Sim >= 5.0) and writes the two activate.d hooks that make the Kit
    # libraries and the torch-bundled libgomp visible. Do not hand-roll this.
    "$ISAACLAB_DIR/isaaclab.sh" --conda "$CONDA_ENV"

    conda activate "$CONDA_ENV"
    set -u

    # `--install` with no argument pulls in every RL framework isaaclab_rl knows
    # about: stable-baselines3, rsl-rl, rl-games and PyPI skrl. Resolving that
    # set makes pip backtrack and quietly upgrade torch to a CUDA 13 build,
    # which Isaac Sim cannot load. This project only uses skrl, and only our own
    # fork of it, so install no frameworks here and add the fork below.
    warn "Installing Isaac Lab (this pulls the ~3 GB torch CUDA wheel set)"
    "$ISAACLAB_DIR/isaaclab.sh" --install none

    # Pin whatever torch Isaac Lab just picked (the version is hardcoded per
    # architecture inside isaaclab.sh) so nothing below can drag in a different
    # CUDA build as a transitive dependency.
    torch_ver=$(python -m pip show torch 2>/dev/null | awk -F': ' '/^Version/{print $2}')
    tv_ver=$(python -m pip show torchvision 2>/dev/null | awk -F': ' '/^Version/{print $2}')
    case "$torch_ver" in
        *+cu*) ;;
        *) die "expected a CUDA build of torch after Isaac Lab install, got '${torch_ver:-nothing}'" ;;
    esac
    CONSTRAINTS=$(mktemp)
    trap 'rm -f "$CONSTRAINTS"' EXIT
    printf 'torch==%s\ntorchvision==%s\n' "$torch_ver" "$tv_ver" > "$CONSTRAINTS"
    export PIP_CONSTRAINT="$CONSTRAINTS"
    warn "pinned torch==$torch_ver for the remaining installs"

    warn "Installing skrl and the MARL extension (editable)"
    python -m pip install -e "$ROOT/external/skrl"
    python -m pip install -e "$ROOT/external/MARL_cooperative_aerial_manipulation_ext/exts/MARL_mav_carry_ext"

    # A transitive dependency swapping torch out is the likeliest way for this
    # environment to end up subtly broken, so check instead of assuming.
    now=$(python -m pip show torch 2>/dev/null | awk -F': ' '/^Version/{print $2}')
    [ "$now" = "$torch_ver" ] || die "torch changed from $torch_ver to $now during install"
    if python -c 'import torch, sys; sys.exit(0 if torch.cuda.is_available() else 1)' 2>/dev/null; then
        warn "torch $torch_ver, CUDA visible"
    else
        warn "torch $torch_ver installed, but CUDA is not visible from this machine"
    fi
    unset PIP_CONSTRAINT
fi

# ------------------------------------------------------- 4. developer docs

if [ "$SKIP_DOCS" -eq 1 ]; then
    say "Skipping developer docs (--skip-docs)"
elif [ -d "$ROOT/dev-docs/.git" ]; then
    say "Updating developer docs"
    git -C "$ROOT/dev-docs" pull --quiet --ff-only || warn "could not fast-forward dev-docs; leaving as-is"
    [ -x "$ROOT/dev-docs/scripts/link-docs.sh" ] && "$ROOT/dev-docs/scripts/link-docs.sh"
else
    say "Fetching developer docs"
    if git clone --quiet "$DEV_DOCS_REPO" "$ROOT/dev-docs" 2>/dev/null; then
        [ -x "$ROOT/dev-docs/scripts/link-docs.sh" ] && "$ROOT/dev-docs/scripts/link-docs.sh"
        warn "developer docs installed in dev-docs/"
    else
        rm -rf "$ROOT/dev-docs"
        warn "no access to castor-dev-docs, skipping."
        warn "This is expected unless you are a member of the FYP-UAV-ENTC-22 org."
    fi
fi

# ------------------------------------------------------- done

say "Setup complete"
cat <<EOF
    Start every session with:

        source $ROOT/env/env.sh

    Then smoke-test training (about a minute):

        cd $ROOT/external/MARL_cooperative_aerial_manipulation_ext
        python scripts/skrl/train.py \\
            --task=Isaac-flycrane-payload-decentralized-hovering-v0 \\
            --headless --num_envs=8 --max_iterations=3 --seed=42 --algorithm=MAPPO
EOF
