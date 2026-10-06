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

## The packages

| Package | Trained on | Rig (`rig.config`) | Flown with |
|---|---|---|---|
| `castor_hover_falcon/v1` | `Isaac-castor-payload-decentralized-hovering-v0`, Falcon drones | `payload_rig_marl.yaml`: 0.5 m disc, 2 m cables | step 0.015 (trained 0.05), velocity filter 0.1 s, point 0.03 m above the tie point |
| `castor_hover_s500/v1` (`DEFAULT`) | `Isaac-castor-s500-payload-decentralized-hovering-v0`, S500 | `payload_rig_marl_s500.yaml`: 0.3 m disc, 3 m cables | the trained settings: step 0.02, speed cap 1 m/s, point at the centre of mass |

A policy only works on the rig it was trained on, and the runner takes the rig's geometry (cable length, anchors)
from the manifest for the lift and the hand-over check. In stack_sim the simulator builds the rig the manifest names
(`rig.config`, from `components/simulation/assets/config`) and checks the manifest's other rig numbers against it:

```bash
make sim-pegasus-ros2 MODEL=castor_hover_falcon/v1          # both default to DEFAULT
components/simulation/stack_sim/stack_sim.sh up --model castor_hover_falcon/v1
```

## What reads the manifest

| Field | Used by |
|---|---|
| `policy.slots[i]` | `castor_policy` on the drone with `team.index = i` |
| `policy.frame_dim`, `history`, `rate_hz`, `point_local` | `castor_policy`: observation width, loop rate, the point given to the policy as the drone's position |
| `flight.*` | `castor_policy`: step scale, speed cap, leash, velocity filter, goal clamp |
| `rig.*` | `castor_policy`: cable span for the hand-over check, lift height; stack_sim: which rig to simulate, and the take-off height (never below the cables' taut height minus a margin) |
| `training.*` | reference only, and the parity check in `components/simulation/tests/marl_raptor/marl_policy.py` |

`castor_common/models.py` resolves a package and turns it into the runner's parameters.

## How a model reaches the runner

The planning image carries this directory at `/opt/castor/models`, copied as the image's last layer: a model change
rebuilds only that layer, and a code change leaves it untouched.

`/var/lib/castor/models` (`CASTOR_MODELS_DIR`, mounted read-only) is searched first, with the same layout. To fly
another model without a new image, put its package there and either write `DEFAULT` there or launch planning with
`model:=<name>/<version>`. Restart the planning container to pick it up.

- **stack_sim**: `components/simulation/stack_sim/stack_sim.sh up` copies the chosen package (`--model
  <name>/<version>` or a path; default: `DEFAULT` here) into each simulated drone's models directory, so stack_sim
  always flies the repo's copy, with no image rebuild.
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
