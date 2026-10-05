# CASTOR

**C**ooperative **A**erial **S**uspended-load **T**ransport with **O**nboard
**R**einforcement learning.

Three quadrotors cooperatively carry a cable-suspended payload, a *flycrane*.
Each drone runs the multi-agent reinforcement learning policy on its onboard
computer, a Raspberry Pi, and sends setpoints to RAPTOR, a neural inner-loop
controller running on the PX4 flight controller. Inter-UAV localisation is done
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
isaac-python <script.py>      # any script that needs Isaac Sim, from anywhere in the repo
```

`env.sh` uses the simulation container (`make sim-image`, see
[docker/README.md](docker/README.md)) when its image exists, and starts it;
otherwise a native Isaac Sim linked by `setup.sh`; otherwise it stops with an
error. `CASTOR_ISAAC=container|native` forces one. Neither an external Isaac
install nor NVIDIA's asset pack is needed with the container.

## Layout

Every piece of source lives under the component that owns it. Each component
becomes one container; submodules sit inside the component that uses them.

```
castor-main/
├── setup.sh                  one-command workspace setup
├── env/env.sh                Isaac for this shell: the simulation container, else the native `castor` env
├── components/
│   ├── vehicle/              flight controller and sensor interfacing
│   │   ├── PX4-Autopilot/    (fork) flight firmware; the RAPTOR neural-policy module
│   │   ├── uwb_firmware/     (castor-localization) UWB DW3000 ranging firmware, STM32
│   │   └── tools/            MAVLink sniffer, RAPTOR goto
│   ├── localization/         state estimation (empty for now)
│   ├── planning/             the MARL policy: training source and onboard runtime
│   │   ├── MARL_cooperative_aerial_manipulation_ext/   (fork) the RL task and training
│   │   └── skrl/             (fork) RL algorithms (MAPPO / IPPO)
│   ├── system/               state machine, behaviour trees, safety, MRM
│   └── simulation/           Isaac Sim based simulation
│       ├── IsaacLab/         (fork) v2.3.0, patched for Isaac Sim 5.1
│       ├── pegasus_simulator/  (fork) PX4 + ROS 2 bridge for Isaac Sim
│       └── tests/            RAPTOR on Pegasus
└── legacy/drone-ops/         retired ArduCopter spike, historical record only
```

Two rules hold across the workspace:

- **Forked repositories keep their upstream names.** IsaacLab, skrl,
  PX4-Autopilot, pegasus_simulator and MARL_cooperative_aerial_manipulation_ext
  are somebody else's projects that we track and patch; renaming them would hide
  that. Only CASTOR's own repositories carry the `castor-` prefix.
- **Each subproject has its own conventions.** Different languages, build
  systems and safety constraints — read the subproject before assuming anything
  carries over.

`vehicle`, `localization`, `planning` and `system` run on every drone's
Raspberry Pi. All five, simulation included, run on a laptop or GPU workstation
during simulation and training.

### Moving from the old `external/` layout

The submodules used to live under `external/`, and their names changed with the
move. In an existing clone, after pulling:

```bash
# external/ now holds only stale checkouts. Check them for unpushed work first:
for d in external/*/; do git -C "$d" status --short --branch; done
rm -rf external                 # only once nothing above needs saving
git submodule sync
git submodule update --init     # fetches each submodule at its new path
./setup.sh                      # rebuilds the conda env against the new paths
```

A fresh clone plus `./setup.sh` gets the same result.

## Containers

Each onboard component runs in its own ROS 2 Jazzy container, all on the host
network and one ROS graph (Fast DDS, domain 20), with a zenoh bridge carrying
selected topics between drones and the ground station. Images are multi-arch
(amd64 and arm64), so the same tag runs on a laptop, a Pi 4 or a Pi 5:
`ghcr.io/fyp-uav-entc-22/castor-{vehicle,localization,planning,system}`.

Robot identity (id, ROS namespace, the policy's team slot, device paths) comes
from one mounted file, `/etc/castor/robot.yaml`; see
[deploy/robot.example.yaml](deploy/robot.example.yaml).

```bash
make help                 # every target
make images-test          # build and test the four images locally
make stack-up             # run them here, against deploy/robot.laptop.yaml
```

- [docker/README.md](docker/README.md): images, dev containers, adding dependencies.
- [deploy/README.md](deploy/README.md): setting up a Pi, robot.yaml, updating (always manual).

## Running a training smoke test

```bash
source env/env.sh
cd components/planning/MARL_cooperative_aerial_manipulation_ext
isaac-python scripts/skrl/train.py \
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
