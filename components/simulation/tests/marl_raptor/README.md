# MARL policy on RAPTOR in Isaac Sim

Two ways to fly the trained flycrane MARL policy on three S500s carrying the payload, with RAPTOR as the inner loop:

- **With PX4** (`px4_rig.py` + `px4_flight.py`): three PX4 SITL instances fly the drones, RAPTOR is PX4's own
  `mc_raptor` mode, and every command and every measurement goes through PX4's uXRCE-DDS link and the CASTOR
  `vehicle` containers. This is the one that matches the aircraft.
- **Without PX4** (`marl_payload.py`): RAPTOR as a Python backend, ground-truth state, one process. A quick check of
  the policy and the sequence.

Both follow the same sequence: take off with slack cables, tension, lift the payload, and only then hand the three
position setpoints to the policy and send it goals.

| File | What it is |
|---|---|
| `px4_rig.py` | PX4 run, simulator side: starts three `px4_sitl_raptor` instances, the S500s with RTK-grade GNSS, cables and payload. Sends the payload pose out, draws the goal. |
| `px4_flight.py` | PX4 run, DDS side: arms, takes off and changes mode through PX4's `vehicle_command`, tensions and lifts, then runs the policy. Interactive or `--auto`. |
| `px4_stack.yml`, `robots/` | Three `vehicle` containers (uXRCE-DDS agent + `vehicle_interface`), one per PX4, and the container `px4_flight.py` runs in. |
| `marl_policy.py` | The policy's side of the chain in NumPy: the 135-value observation, the exported actor (ONNX), the setpoint integrator. Run it on its own to check all three against a trace from the training environment. |
| `marl_payload.py` | The run without PX4: scripted take-off, tension and lift, the hand-over check, then the policy and its goals. |
| `marl_scene.py` | The goal marker both runs draw. |
| `models/` | The policy flown here last: the Falcon-trained one (`Isaac-castor-payload-decentralized-hovering-v0`), as three exported actors and the reference trace. Exporting another policy replaces it, see below. |

RAPTOR is the unchanged `RaptorBackend` of [`../raptor`](../raptor).

## Which policy: `--trained_on`

A policy has to be flown on the rig, and with the setpoint arithmetic, of the task it was trained on. All three
runners take `--trained_on`, which sets these together (`TRAINED_ON` in `marl_policy.py`):

| | `--trained_on falcon` (default) | `--trained_on s500` |
|---|---|---|
| Training task | `Isaac-castor-payload-decentralized-hovering-v0` | `Isaac-castor-s500-payload-decentralized-hovering-v0` |
| Rig | [`payload_rig_marl.yaml`](../../assets/config/payload_rig_marl.yaml): 0.5 m disc, 2 m cables | [`payload_rig_marl_s500.yaml`](../../assets/config/payload_rig_marl_s500.yaml): 0.3 m disc, 3 m cables |
| The drone's position, for the policy | 0.03 m above the cable tie point | the body's centre of mass |
| Step scale | 0.025 without PX4, 0.015 with (training: 0.05, see below) | 0.02, as trained |
| Setpoint speed cap | none | 1.0 m/s, as trained |
| Velocity feedforward filter (PX4 run) | 0.1 s | none, as trained |

Both rigs carry a 0.4 kg disc with anchors 120 degrees apart and the drones facing outward. `--rig`, `--step_scale`,
`--max_speed` and `--velocity_filter` override one setting at a time. In the PX4 run give `px4_rig.py` and
`px4_flight.py` the same `--trained_on`; the flight node warns if they differ.

The numbers measured further down are all from the Falcon-trained policy.

## Setup (once)

```bash
source env/env.sh
cd components/simulation/tests/marl_raptor
python -m pip install --target .deps --no-deps onnxruntime flatbuffers pymavlink
# for the PX4 run: the same, for the Python inside the ROS containers (3.12)
python -m pip install --target .deps-py312 --no-deps --python-version 3.12 --only-binary=:all: \
    --platform manylinux_2_27_x86_64 --platform manylinux_2_28_x86_64 onnxruntime flatbuffers
```

`--target .deps --no-deps` keeps these out of the `castor` env on purpose: pip inside that env can see, and remove,
packages that belong to the Isaac Sim install.

`models/` holds the Falcon-trained policy. To put another one there, in the MARL repo, with the Python that trained it:

```bash
RUN=logs/skrl/mappo_castor_hover/<run>
OUT=<castor-main>/components/simulation/tests/marl_raptor/models
python scripts/tools/export_policy_onnx.py $RUN/checkpoints/best_agent.pt --out $OUT
python scripts/tools/capture_policy_trace.py --checkpoint $RUN/checkpoints/best_agent.pt --out $OUT/policy_trace.npz
```

`export_policy_onnx.py` writes one actor per one-hot slot, `policy_falcon1.onnx` to `policy_falcon3.onnx`, each with
its input scaler inside. Drone `i` of the rig (cable `i`, anchor at `i * 120` degrees) loads `policy_falcon<i + 1>`.

For a policy of the S500 task the run is under `logs/skrl/mappo_castor_s500_hover/`, and the trace has to be told the
task: add `--task Isaac-castor-s500-payload-decentralized-hovering-v0` to `capture_policy_trace.py`. The trace is
recorded without that task's observation noise and delay, and carries the task's step scale and speed cap, which
`python marl_policy.py` then checks against.

## Run with PX4

Needs a PX4 checkout with `px4_sitl_raptor` built (`make px4_sitl_raptor`; default `~/Documents/Repos/PX4-Autopilot`,
`--px4` to change) and the vehicle image (`make vehicle-image` in the repo root).

```bash
# terminal 1: the simulator and the three PX4s
source env/env.sh
cd components/simulation/tests/marl_raptor
python px4_rig.py                           # --trained_on s500 for a policy of the S500 task

# terminal 2: one vehicle container per PX4, then the flight node
cd components/simulation/tests/marl_raptor
docker compose -f px4_stack.yml up -d
docker compose -f px4_stack.yml run --rm flight
# for a policy of the S500 task:
docker compose -f px4_stack.yml run --rm flight python3 px4_flight.py --trained_on s500
```

The flight node waits for the three PX4s, then takes one word at a time:

| You type | What happens | Who flies |
|---|---|---|
| `takeoff` | PX4 arms and takes off in its take-off mode, to a hover with the cables still slack | PX4's own controllers |
| `raptor` | PX4 switches to the RAPTOR mode and holds position | RAPTOR, no setpoints |
| `tension` | setpoints rise at 0.15 m/s: cables go taut, the payload lifts to 1 m | RAPTOR, scripted setpoints |
| `rl` | hand-over check, then the policy takes the setpoints and holds the payload | RAPTOR, MARL setpoints |
| `goal 0.6 0.4 1.2` or `goal 0.6 0.4 1.2 0 0 30` | the payload flies there; x y z in metres, then roll pitch yaw in degrees | RAPTOR, MARL setpoints |
| `stop` | setpoints freeze | RAPTOR |
| `land` | PX4 land mode | PX4's own controllers |
| `status` | modes, positions, cable spans | |

A word that does not fit the current step is refused with a message. Goals are clamped to the box the policy was
trained on (x and y within 1 m of the origin, z from 0.5 to 1.5 m) and drawn in the simulator as a green disc.

For an unattended run, which exits 1 if the hand-over is refused or a goal is missed:

```bash
docker compose -f px4_stack.yml run --rm flight python3 px4_flight.py --auto --goal=0.6,0.4,1.2,0,0,30
docker compose -f px4_stack.yml down      # when finished
```

### What goes through PX4, and what does not

| | Path |
|---|---|
| Drone position, velocity, attitude, body rates | PX4's estimator -> uXRCE-DDS -> `vehicle_interface` -> `<ns>/vehicle/odom` |
| Armed, mode | PX4 `vehicle_status` -> `<ns>/vehicle/state` |
| Where each PX4's local frame is on the globe | PX4 `vehicle_local_position` (`ref_lat`, `ref_lon`, `ref_alt`) |
| Arm, take off, RAPTOR mode, land | `vehicle_command` over uXRCE-DDS, no MAVLink |
| Setpoints | `<ns>/vehicle/setpoint` -> `vehicle_interface` -> PX4 `trajectory_setpoint` -> `mc_raptor` |
| Payload pose | the simulator, with 1 cm and 0.5 degree noise, over UDP, republished as `/team/payload/odom` |

**Frames.** Each PX4 has its own local frame, starting where its estimator did. The flight node turns each frame's
geodetic origin into an offset from the datum (the world origin's coordinates), so all three drones and the payload
are in one east-north-up world frame before the policy sees them, and turns setpoints back into each drone's local
frame. Goals are in that world frame.

**GNSS.** The simulated receivers are set to RTK-grade noise (2 cm horizontal, 3 cm vertical, fix type 6) and PX4
uses GNSS for height. Cable spans computed from those positions come out within 1 to 2 cm of the truth.

**Steadiness.** `px4_flight.py` prints, for every phase, how much the drones tilted and how fast they rolled and
pitched, from PX4's own attitude and rates, and for every goal when the payload settled (within 5 cm and 3 degrees
for good). RAPTOR holding by itself is steady, about 2 deg/s. With the policy on top at training's settings the
drones rock at about 2 Hz while the payload stays on its goal.

It is not measurement noise: with the payload noise switched off (`px4_rig.py --payload_noise 0`) the rocking is
the same, and a low-pass filter on the payload pose (`--payload_filter`) changes nothing. It is the loop between the
policy and the drone. The policy learned how hard to move a setpoint against the Falcon with instant, exact state;
here the S500 answers more slowly and the state arrives through PX4's estimator and the DDS link, so the same push
overshoots and the policy keeps correcting. The velocity feedforward (the setpoint's step divided by 20 ms) is the
fast path of that loop. Measured with PX4:

| `--step_scale` | `--velocity_gain` | `--velocity_filter` | roll/pitch rate rms, holding / moving | 60 degree turn settled |
|---|---|---|---|---|
| 0.025 | 1 | 0 | 34 / 69 deg/s | not measured |
| 0.025 | 1 | 0.1 s | 20 / 71 deg/s | after 9.3 s |
| 0.015 | 1 | 0 | 17 / 17 deg/s | not within 12 s (3.3 deg left) |
| 0.015 (default) | 1 (default) | 0.1 s (default) | 8 / 9 deg/s | at about 12 s (3.4 deg left) |
| 0.015 | 0 | 0 | 2 / 2 deg/s | not within 12 s (4 to 7 deg left) |

The defaults keep the velocity feedforward, smoothed over 0.1 s, with a smaller step. Faster settings rock more,
steadier ones align the payload more slowly; there is no setting of this policy that is both. A policy trained on
the S500 with the estimator's and the link's delay in the loop is the way to get both.

**Where this differs from the aircraft.** `px4_flight.py` is one process standing in for the ground station and for
the three onboard policy runners. On the aircraft each drone runs its own policy in the `planning` container, the
world-frame conversion lives in `localization`, and the sequence is gated by `system`. The vehicle image's
`px4_msgs` come from the PX4 fork in this repository; the messages used here are identical in upstream PX4.

## Run without PX4

```bash
python marl_policy.py                       # pipeline check against the trace, no Isaac Sim; exits 1 on a mismatch

python marl_payload.py --headless                                        # one default goal
python marl_payload.py --headless --goal=0.6,0.4,1.2,0,0,30 --goal=-0.5,0.5,0.9
python marl_payload.py --headless --no_rl                                # take-off, tension and lift only
python marl_payload.py                                                   # with the window: the keyboard moves the goal
python marl_payload.py --goal=0.6,0.4,1.2 --keep_open                    # scripted goal, then the keyboard
python marl_payload.py --headless --trained_on s500                      # a policy of the S500 task
```

A goal is `x,y,z` in metres in the world frame, optionally followed by `roll,pitch,yaw` in degrees; each gets
`--goal_time` seconds (12). The goal is drawn as a translucent green disc with its three axes, and each drone's
setpoint as a yellow ball. With the window, click an empty part of the viewport and use I/K (x), J/L (y), U/O (z) to
move the goal 0.25 m a press, H to return it to where the policy took over. The keyboard keeps the goal inside
the box the policy was trained on: x and y within 1 m of the origin, z from 0.5 to 1.5 m.

## The sequence

| Phase | Who moves the setpoints | Ends when |
|---|---|---|
| ground | nobody: RAPTOR holds the spawn point | 1 s |
| take-off | script: straight up to 0.10 m below where the cable goes taut | `--takeoff_speed` (0.5 m/s mean) |
| tension and lift | script: all three rise together | `--lift_speed` (0.15 m/s), payload at `--lift_height` (1.0 m) |
| hold | script | 4 s, then the hand-over check |
| RL | the policy, holding the payload where it is | 3 s |
| goals | the policy | `--goal_time` each |

The hand-over check refuses to start the policy unless every cable is within 3 cm of its length and the payload is at
least 0.2 m off the ground; the policy was only ever trained from that state. On a real aircraft the first three phases
are the operator's.

## How it maps onto training

- **Rates.** Physics 400 Hz, RAPTOR every physics step (GRU at 100 Hz), the policy every 8th step (50 Hz). Training
  stepped RAPTOR at a plain 100 Hz.
- **Frames.** One world frame for everything, ENU. Linear and angular velocity both in the world frame; Pegasus
  reports body rates, so they are rotated first.
- **The drone's position.** The Falcon task reports a point 0.03 m above where the Falcon's cable ties on. The S500
  carries its cable on the rod under the airframe, so a Falcon-trained policy is given the point 0.03 m above the tie
  point, about 0.16 m below the body origin. The S500 task reports the body's centre of mass, 0.025 m below the body
  origin, and a policy trained there is given that. Either way the velocity is that point's, and RAPTOR's target is
  shifted back by the same offset.
- **Setpoint.** `action * step_scale` added to a persistent setpoint, the step cut to the speed cap if the task has
  one, velocity feedforward `increment / 0.02 s`, setpoint kept within 1.5 m of the drone, yaw held at the drone's formation heading. Seeded from the drone at
  hand-over.
- **Step scale, Falcon-trained policy.** Training used 0.05 m per unit action; `--step_scale` defaults to 0.025 here. With 0.05 the policy
  and the S500 form a loop that swings the setpoints at about 2 Hz: the drones rock 10 to 25 degrees while holding
  still, though the payload stays on its goal. RAPTOR alone holds the same formation at a steady 2.5 degrees, and
  stepping RAPTOR at training's plain 100 Hz or adding training's motor lag changes nothing, so the difference is
  the airframe: the policy learned its gain against the Falcon, which answers a setpoint change faster than the
  S500 does. Measured at a held goal:

  | `--step_scale` | drone tilt (mean) | roll/pitch rate (max) | payload error |
  |---|---|---|---|
  | 0.05 (training) | 12.6 deg | 248 deg/s | 3.5 cm |
  | 0.035 | 7.0 deg | 136 deg/s | 1.0 cm |
  | 0.025 (default) | 2.2 deg | 7 deg/s | 0.7 cm |
  | 0.0175 | 2.2 deg | 2 deg/s | 0.7 cm, slower to arrive |

  Halving the scale also halves the speed a unit action asks for, to 1.25 m/s. It is a setting for this policy on
  this airframe; a policy trained on the S500 model should fly at the scale it was trained with.

## Not covered here

Ground-truth state for drones and payload, no PX4, no ROS: this checks the policy and the sequence, not the onboard
stack. Goals arrive as steps; a step of a metre still makes the drones tilt about 25 degrees on the way, which is
fine here and not something to do with a real aircraft.

`--diag` switches off or changes one part of the chain at a time (`no_vel_ff`, `filter=A`, `point=Z`,
`raptor_100hz`); they are for finding where a difference from training comes from, not for flying.
