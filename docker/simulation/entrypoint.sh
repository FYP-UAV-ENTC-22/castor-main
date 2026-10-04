#!/bin/bash
# Simulation container entrypoint.
#
# CASTOR_ASSET_ROOT  a local Isaac asset pack mounted read-only (for example
#                    /mnt/isaac/isaacsim_assets/Assets/Isaac/5.1). Kit only takes
#                    the asset root from its persistent settings (an OMNI_KIT_*
#                    variable can't express asset_root), so it is written into
#                    each Kit app's user.config.json here. Unset: NVIDIA's S3.
set -euo pipefail

if [ -n "${CASTOR_ASSET_ROOT:-}" ]; then
    if [ -d "$CASTOR_ASSET_ROOT" ]; then
        for app in "Isaac-Sim" "Isaac-Sim Python" "Isaac-Sim Full"; do
            for base in /isaac-sim/kit/data/Kit "$HOME/.local/share/ov/data/Kit"; do
                f="$base/$app/5.1/user.config.json"
                mkdir -p "$(dirname "$f")"
                /isaac-sim/python.sh - "$f" "$CASTOR_ASSET_ROOT" <<'PY'
import json, os, sys
path, root = sys.argv[1], sys.argv[2]
cfg = json.load(open(path)) if os.path.exists(path) else {}
ar = cfg.setdefault("persistent", {}).setdefault("isaac", {}).setdefault("asset_root", {})
ar.update({"default": root, "cloud": root, "nvidia": root, "timeout": 5.0})
json.dump(cfg, open(path, "w"), indent=2)
PY
            done
        done
    else
        echo "entrypoint: CASTOR_ASSET_ROOT=$CASTOR_ASSET_ROOT is not mounted; assets come from S3" >&2
    fi
fi

exec "$@"
