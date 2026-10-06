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

It flies the mode the system layer asks for on `<ns>/planning/command`, while
every input that mode needs is fresh: `steady` holds a fixed setpoint, `lift`
climbs until the payload hangs at the model's lift height, `policy` runs the
MARL policy; `off` (or a stale command) publishes nothing and RAPTOR holds
position. In every mode it reports the hand-over check (own cable taut, payload
clear of the ground) from the model's rig geometry.
World-frame state comes from `own_state_prefix` / `payload_state_prefix`, for now
the simulator's ground truth. `<ns>/planning/status` reports whether it is active,
why not, and the goal errors.

The policy is a versioned model package from [`models/`](../../models) at the
repo root, baked into the planning image and overridable by mounting
`/var/lib/castor/models`; its manifest sets the runner's parameters. See
[`models/README.md`](../../models/README.md). Besides what training did, the
manifest says how to fly the policy on this airframe (`flight.hpp`): the point
on the drone the policy calls its position (its setpoint is moved back to the
body origin for RAPTOR), the step scale, an optional speed cap, a low-pass on
the velocity feedforward, and the box goals are clamped into.
