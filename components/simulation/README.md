# simulation

Isaac Sim based simulation, used for training and for running the rest of the
stack against simulated drones on a laptop or GPU workstation. Never runs on a
Pi.

| Path | What it is |
|---|---|
| [`IsaacLab/`](IsaacLab/) | Fork. Isaac Lab v2.3.0 for Isaac Sim 5.1. `_isaac_sim` is a symlink to the Isaac Sim install, made by `setup.sh`. |
| [`pegasus_simulator/`](pegasus_simulator/) | Fork. Multirotor simulation and a PX4/ROS 2 bridge as an Isaac Sim extension. |
| [`assets/`](assets/) | Generated USD assets for CASTOR's own hardware: the Holybro S500 quadrotor and the N-drone cable-suspended payload rig, built from YAML. |
| [`sil/`](sil/) | Software in the loop: one onboard stack per simulated drone (`sil.sh`), flown by Pegasus + PX4 SITL (`sil_pegasus.py`). See [docker/README.md](../../docker/README.md#software-in-the-loop). |
| [`tests/raptor/`](tests/raptor/) | Flies PX4's RAPTOR policy on Pegasus quadrotors in Isaac Sim, alone, in formation, or carrying the payload rig. |
