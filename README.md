# CASTOR

**C**ooperative **A**erial **S**uspended-load **T**ransport with **O**nboard
**R**einforcement learning.

Three quadrotors cooperatively carry a cable-suspended payload — a *flycrane* —
using a multi-agent reinforcement learning policy that runs **on the flight
controller** rather than on a companion computer. Inter-UAV localisation is done
with UWB ranging, so the system needs no external motion-capture rig.

ENTC Batch 22, Group 25 final year project.

---

## Quick start

```bash
git clone git@github.com:FYP-UAV-ENTC-22/castor-main.git
cd castor-main
./setup.sh
```

`setup.sh` fetches every submodule, links an existing Isaac Sim install, builds
the `castor` conda environment, and — if you are an org member — pulls the
private developer docs. It does **not** install Isaac Sim; point it at one you
already have:

```bash
ISAACSIM_PATH=/path/to/isaacsim ./setup.sh
```

Useful flags: `--skip-env` (submodules and docs only), `--skip-docs`, `--help`.

Then, in every new shell:

```bash
source env/env.sh
```

## Layout

```
castor-main/
├── setup.sh              one-command workspace setup
├── env/env.sh            activates the `castor` conda env
├── components/           CASTOR's own runtime components
│   ├── localization/     UWB (DW3000) ranging firmware — STM32 Nucleo
│   ├── simulation/       reserved
│   ├── vehicle/          reserved — flight controller and sensor interfacing
│   ├── control/          reserved
│   └── system/           reserved — state machine, safety, MRM
├── external/             third-party repositories, forked but never renamed
│   ├── IsaacLab/         v2.3.0, patched for Isaac Sim 5.1
│   ├── MARL_cooperative_aerial_manipulation_ext/   the RL task and training
│   ├── skrl/             RL algorithms (MAPPO / IPPO)
│   ├── PX4-Autopilot/    flight firmware; the RAPTOR neural-policy module
│   └── pegasus_simulator/  PX4 + ROS 2 bridge for Isaac Sim
└── legacy/drone-ops/     retired ArduCopter spike — historical record only
```

Two rules hold across the workspace:

- **Forked repositories keep their upstream names.** Anything under `external/`
  is somebody else's project that we track and patch; renaming it would hide
  that. Only CASTOR's own repositories carry the `castor-` prefix.
- **Each subproject has its own conventions.** Different languages, build
  systems and safety constraints — read the subproject before assuming anything
  carries over.

The five directories under `components/` are the units that will later become
separate containers. `localization`, `vehicle`, `control` and `system` are the
ones intended to run on the Raspberry Pi alongside the flight controller; all
five run on a laptop or GPU workstation during simulation and training.

## Running a training smoke test

```bash
source env/env.sh
cd external/MARL_cooperative_aerial_manipulation_ext
python scripts/skrl/train.py \
    --task=Isaac-flycrane-payload-decentralized-hovering-v0 \
    --headless --num_envs=8 --max_iterations=3 --seed=42 --algorithm=MAPPO
```

There is no test suite. Verification means a short headless run and reading the
log.

## Hardware status

**No flight-ready aircraft currently exists.** The earlier ArduCopter rig is
retired (see `legacy/drone-ops/`). Three Holybro Pixhawk 6C mini units are on
order. Anything that arms a vehicle or spins motors is props-off until a written
bench procedure exists for the new hardware.

## Developer docs

Design notes, per-repository developer guides, reports and design material live
in a separate private repository, `castor-dev-docs`, readable by org members
only. `setup.sh` clones it into `dev-docs/` when you have access and skips it
silently when you do not — a public clone is complete without it.
