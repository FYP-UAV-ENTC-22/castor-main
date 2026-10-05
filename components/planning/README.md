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

## Onboard runner (`castor_policy`)

`policy_runner` runs the raptor_v1.1 flycrane policy: each step it builds the
training env's observation (payload pose, its one-hot, own pose and velocities,
goal error; three frames), runs the ONNX model and turns the 3-D output into a
position increment, exactly as the env does (`setpoint_step_scale`,
`setpoint_leash`, target yaw = the drone's yaw when the policy starts). The
setpoint goes to `<ns>/vehicle/setpoint` in the vehicle's own odometry frame, and
the vehicle component hands it to RAPTOR on the FC.

It flies only while the system layer enables it on `<ns>/planning/command` and
every input is fresh; otherwise it publishes nothing and RAPTOR holds position.
World-frame state comes from `own_state_prefix` / `payload_state_prefix`, for now
the simulator's ground truth. `<ns>/planning/status` reports whether it is active,
why not, and the goal errors. Export a checkpoint with
`tools/export_policy.py` and mount it at `/var/lib/castor/models/policy.onnx`.
