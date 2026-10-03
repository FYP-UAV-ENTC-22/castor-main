# planning

The multi-agent flycrane policy: where it is trained, and the onboard node that
runs it on each drone's Raspberry Pi.

| Path | What it is |
|---|---|
| [`MARL_cooperative_aerial_manipulation_ext/`](MARL_cooperative_aerial_manipulation_ext/) | Fork. The Isaac Lab flycrane tasks and the MAPPO/IPPO training scripts. |
| [`skrl/`](skrl/) | Fork. The RL algorithms, installed editable into the `castor` env. |

Training needs Isaac Sim and an NVIDIA GPU, so it never runs on a Pi; it uses
Isaac Lab from [`../simulation/IsaacLab`](../simulation/IsaacLab). The Pi image
for this component contains only the onboard policy runtime, never the training
stack.

RAPTOR, the inner-loop policy on the flight controller, lives in PX4:
[`../vehicle/PX4-Autopilot/src/modules/mc_raptor`](../vehicle/PX4-Autopilot/src/modules/mc_raptor).
