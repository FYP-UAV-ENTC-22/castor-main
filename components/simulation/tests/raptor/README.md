# RAPTOR in Isaac Sim

Flies the RAPTOR foundation policy (the network PX4's `mc_raptor` module runs,
`components/vehicle/PX4-Autopilot/src/modules/mc_raptor/blob/policy.tar`) on Pegasus quadrotors in Isaac Sim,
without PX4. One `RaptorBackend` per vehicle, each with its own GRU state.

| File | What it is |
|---|---|
| `raptor_policy.py` | PyTorch port of the checkpoint: Dense(22→16, ReLU) → GRU(16) → Dense(16→4). Batched. Run it on its own to check it against the example sequence stored in `policy.tar`. |
| `raptor_backend.py` | `RaptorBackend`, the Pegasus backend that runs the policy the way `mc_raptor` does, plus the Iris trajectories. Shared by both runners. |
| `raptor_pegasus.py` | Single-vehicle and formation runner on Pegasus' Iris: waypoint, hover and Lissajous references, `--num_vehicles N`. |
| `raptor_payload.py` | Payload runner: N drones (S500 or Iris) carrying a cable-suspended payload, built from `../../assets/config/`. Hover and slow waypoint runs, cable release, payload and cable logging. |
| `sim_loop.py` | Stepping loop that gives the same physics with and without the window (see below). |
| `keyboard_teleop.py` | Keyboard control for GUI runs: move the formation, fire the release mechanisms. |
| `compare_logs.py` | Step-by-step difference between two runs, e.g. GUI vs headless. |

## Setup (once)

Nothing in the simulation container (`source env/env.sh` picks it when it is
built). With a native Isaac Sim only: Pegasus imports `pymavlink` even when no
PX4 is used. Install it next to the scripts rather than into the `castor` env:

```bash
CASTOR_ISAAC=native source env/env.sh
python -m pip install --target components/simulation/tests/raptor/.deps --no-deps pymavlink
```

## Run

```bash
source env/env.sh
cd components/simulation/tests/raptor

isaac-python raptor_policy.py                     # port check, no Isaac Sim needed; exits 1 on mismatch

isaac-python raptor_pegasus.py --trajectory waypoints --num_vehicles 3       # with the GUI
isaac-python raptor_pegasus.py --headless --trajectory lissajous             # headless
isaac-python raptor_pegasus.py --headless --trajectory hover --spawn_z 0.07  # take off from the ground
```

`--physics_hz` defaults to 400 (the `IMU_GYRO_RATEMAX` used on real flight
controllers), so the GRU advances every 4th physics step.

### Payload scenario

The rig (airframe, number of drones, payload, cable model and length, mount height, release mechanisms) comes from
[`../../assets/config/payload_rig.yaml`](../../assets/config/payload_rig.yaml) and
[`s500.yaml`](../../assets/config/s500.yaml); see [`../../assets/README.md`](../../assets/README.md). Any key can be
overridden with `--set` (rig) or `--vset` (vehicle).

```bash
# headless
isaac-python raptor_payload.py --headless --trajectory hover                          # 3 x S500, distance-joint cables
isaac-python raptor_payload.py --headless --trajectory waypoints --set cable.model=rope
isaac-python raptor_payload.py --headless --set num_drones=4 --set cable.length=1.2
isaac-python raptor_payload.py --headless --set vehicle=iris                          # Pegasus' Iris instead
isaac-python raptor_payload.py --headless --release 8:0:drone                         # release cable 0 at the drone, t = 8 s
isaac-python raptor_payload.py --headless --no_payload --set num_drones=1             # the airframe alone

# with the window: the camera frames the formation; --keep_open pauses at the end and leaves the window open
isaac-python raptor_payload.py --trajectory waypoints --set cable.model=rope --keep_open
isaac-python raptor_payload.py --trajectory hover --follow --keep_open                # camera follows the payload
```

### Playing around in the window

```bash
isaac-python raptor_payload.py --trajectory manual                                 # fly the formation with the keyboard
isaac-python raptor_payload.py --trajectory waypoints --keep_open                  # scripted run, then press Play and fly
```

Keys (click an empty part of the viewport first, so no prim is selected): I/K move the formation along x, J/L along
y, U/O up and down (arrows and PgUp/PgDn work too), H returns to the start position. Each press moves the target
0.25 m and the drones' target glides there at 0.4 m/s. 1..9 release cable 0..8 at the drone end, Shift+1..9 at the
payload end (only where `release.location` puts a mechanism). Releases are applied between physics steps, on the next
step after the key press. Release keys also work during scripted runs. In `manual` mode the run lasts until you close
the window (or `--duration`), and the log is written then.

`hover` holds the spawn formation for 15 s. `waypoints` moves the whole formation with minimum-jerk segments
(1 m along x, then 1 m along y while climbing 0.3 m, then back; 6 s per move, 4 s holds, at most ~0.45 m/s), 34 s
in total. The trajectory is a formation offset: each drone's RAPTOR target is its own spawn point plus that offset,
at its own yaw. RAPTOR is not told about the payload.

Logs (`runs/payload_<trajectory>_<cable>_n<N>.npz`) hold each drone's RAPTOR log (`v<i>_*`, as in
`raptor_pegasus.py`), the rig's state per physics step (`rig_*`: payload pose and velocity, cable end points,
end-to-end distance and stretch, tension, attachment state, rope end-joint gaps) and a JSON `meta` entry with the
resolved configs, the layout and the release events. The run prints a summary: tracking error per drone, payload
offset and tilt, cable stretch, slack time and tension.

Distance-joint cables are drawn as thin cylinders, moved every rendered frame (GUI only; they have no physics).

### GUI and headless give the same physics

`world.step(render=True)` calls `app.update()`, which advances a whole rendering frame (1/60 s, about 6.7 physics
steps at 400 Hz), so a loop that counted steps ran ~6.7× longer with the window. Both runners now take exactly one
PhysX step per `world.step(render=False)` in both modes, loop until `world.current_time` reaches `--duration`, and
redraw with `world.render()` every `physics_hz / render_hz` steps (`--render_hz`, default 50, i.e. every 8 steps).
`world.render()` runs `app.update()` with `/app/player/playSimulations` switched off, so it never steps physics.
`world.reset()` itself takes 2 physics steps, so the loop takes `duration × physics_hz − 2` and the logs hold exactly
`duration × physics_hz` entries. With the window the run is paced to real time unless `--fast` (it is slower than
real time anyway with three or more drones). Check a pair of runs with:

```bash
python compare_logs.py runs/a.npz runs/b.npz
```

## How it maps onto `mc_raptor`

- **Observation** (22): position error and velocity error in the target-yaw frame,
  clipped to ±0.5 m and ±1 m/s; rotation matrix of target⁻¹ · attitude; body
  rates; previous action. Built in ENU/FLU here, which is equivalent to PX4's
  NED→FLU conversion because every term is relative to the target yaw.
- **Timing**: called every physics step. The GRU state only advances every
  `round(physics_hz / 100)` calls; the calls in between evaluate from the
  current state without advancing it, and the previous-action input is the mean
  of the actions since the last advancing step. This is the rl_tools L2F executor.
- **Output**: `(a + 1) / 2`, remapped from Crazyflie to PX4 quad-X order
  (`px4 = cf[[0, 2, 3, 1]]`), then Pegasus' Iris mapping `ω = 1000 u + 100 rad/s`.
  The S500 asset keeps that mapping and sets its thrust constant so `ω = 1100 rad/s` gives the configured
  full-throttle thrust. The runners check at startup that the rotors sit in quad-X positions.

## Not covered here

Ground-truth state (no EKF2, no sensor noise), no PX4 modes or failsafes, and instantaneous motors unless
`--vset propulsion.motor_time_constant=...`. For the real firmware in the loop, build `px4_sitl_raptor` and use
Pegasus' PX4 backend instead.

## Gotchas

- Kit catches uncaught Python exceptions and still exits 0; the runners exit 1
  on an exception themselves.
- `SimulationApp.close()` hard-exits, dropping buffered stdout when it goes to a
  file; the runners switch stdout to line buffering.
- The `omni.kit.test` / `CXXABI_1.3.15` / `omni.graph ... tests` import errors at startup come from
  test-only extensions and do not affect the run.
