# Policy models

Every policy that flies, in simulation or on the aircraft, is a versioned package here:

```
models/
├── DEFAULT                       <name>/<version> flown when nothing else is chosen
└── <name>/<version>/
    ├── model.yaml                manifest: where it came from, what it expects, how to fly it
    ├── policy_<agent>.onnx       one exported actor per team slot, input scaler inside
    ├── policy_<agent>_test.npz   test vectors from the export (not shipped in the image)
    └── policy_trace.npz          a trace recorded in the training env (not shipped in the image)
```

A committed version never changes. A re-export, a retrain or a new flight setting is a new version directory;
`DEFAULT` then moves to it in its own commit. Identical files (the per-slot actors of a shared policy) are stored
once by git. Checkpoints (`.pt`) stay in the MARL repo's logs, not here.

## What reads the manifest

| Field | Used by |
|---|---|
| `policy.slots[i]` | `castor_policy` on the drone with `team.index = i` |
| `policy.frame_dim`, `history`, `rate_hz`, `point_local` | `castor_policy`: observation width, loop rate, the point given to the policy as the drone's position |
| `flight.*` | `castor_policy`: step scale, speed cap, leash, velocity filter, goal clamp |
| `rig.*` | `castor_policy`: cable span for the hand-over check, lift height; `sil.sh`: take-off height |
| `training.*` | reference only, and the parity check in `components/simulation/tests/marl_raptor/marl_policy.py` |

`castor_common/models.py` resolves a package and turns it into the runner's parameters.

## How a model reaches the runner

The planning image carries this directory at `/opt/castor/models`, copied as the image's last layer: a model change
rebuilds only that layer, and a code change leaves it untouched.

`/var/lib/castor/models` (`CASTOR_MODELS_DIR`, mounted read-only) is searched first, with the same layout. To fly
another model without a new image, put its package there and either write `DEFAULT` there or launch planning with
`model:=<name>/<version>`. Restart the planning container to pick it up.

- **SIL**: `components/simulation/sil/sil.sh up` copies the chosen package (`--model <name>/<version>` or a path;
  default: `DEFAULT` here) into each simulated drone's models directory, so SIL always flies the repo's copy, with no
  image rebuild.
- **Pi**: the image's copy flies by default. `deploy/pi/install.sh` creates `/var/lib/castor/models` for overrides.

A bare `/var/lib/castor/models/policy.onnx` with no manifest still loads, with the runner's built-in defaults.

## Adding a version

In the MARL repo, with the Python that trained it:

```bash
RUN=logs/skrl/<experiment>/<run>
OUT=<castor-main>/models/<name>/<version>
python scripts/tools/export_policy_onnx.py $RUN/checkpoints/best_agent.pt --out $OUT
python scripts/tools/capture_policy_trace.py --checkpoint $RUN/checkpoints/best_agent.pt --out $OUT/policy_trace.npz
```

Then copy the previous version's `model.yaml`, update `source` (MARL commit, task, run), `policy.slots` (with
`sha256sum`), and `rig`/`flight` if they changed. Check the export against the trace:

```bash
python components/simulation/tests/marl_raptor/marl_policy.py --models models/<name>/<version>
```
