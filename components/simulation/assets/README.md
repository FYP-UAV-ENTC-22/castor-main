# Simulation assets

Generated USD assets for CASTOR's own hardware, built from YAML so every dimension is in one reviewed place:

- **`s500`**: a Holybro S500 V2 quadrotor (480 mm wheelbase, 9450 propellers, 2216 KV920 on 4S) with its two-pole
  T landing gear and the stock battery rails. It has the same prim layout as Pegasus' Iris (`/vehicle`
  articulation root, `body`, `rotor0..3` with `joint0..3` in PX4 quad-X order), so Pegasus' `Multirotor` and
  `RaptorBackend` fly it unchanged.
- **payload rig**: N drones (N >= 1) carrying one cylindrical payload on cables, flycrane-style. Anchors sit at equal
  spacing around the payload (circumference / N); each cable ties to the centre of a cross rod clamped between the
  two landing-gear poles, at a configurable height; release mechanisms can sit at either end.

| Path | What it is |
|---|---|
| [`config/s500.yaml`](config/s500.yaml) | Airframe: frame, landing gear, mass budget, propulsion. Every number is tagged `[pub]`, `[photo]` or `[est]`. |
| [`config/payload_rig.yaml`](config/payload_rig.yaml) | Rig: number of drones, payload, formation, mount height, cable model, release mechanisms. |
| [`castor_assets/config.py`](castor_assets/config.py) | Typed loading and validation. Unknown keys are errors. |
| [`castor_assets/geometry.py`](castor_assets/geometry.py) | Frame geometry, mass budget to inertia, formation layout and clearance checks (numpy only). |
| [`castor_assets/usd_build.py`](castor_assets/usd_build.py) | USD authoring: the S500 asset, the rig (into a live stage or a standalone file). |
| [`castor_assets/flycrane.py`](castor_assets/flycrane.py) | The MARL training asset (flycrane) for the S500 and this rig, as URDF; S500 numbers for the training controllers. |
| [`castor_assets/runtime.py`](castor_assets/runtime.py) | The rig in a running sim: state log, cable tension, distance-cable drawing, release API. |
| [`build_assets.py`](build_assets.py) | CLI that writes `generated/s500.usd` and a standalone rig file. |
| `generated/` | Build output, gitignored. Rebuilt on every run of `build_assets.py` or stack_sim's simulator. |

## Build and look at it

```bash
source env/env.sh
cd components/simulation/assets
python build_assets.py --pin_drones                       # generated/s500.usd + generated/rig_s500_n3_distance.usd
python build_assets.py --set num_drones=4 --set cable.model=rope --pin_drones
python build_assets.py --vset mass.total=1.62             # e.g. a measured all-up mass
```

Open `generated/rig_*.usd` in Isaac Sim (File > Open) and press Play. `--pin_drones` fixes each drone to the world,
so the payload just hangs; without it nothing drives the rotors and everything falls. Distance-joint cables
(`cable.model: distance`) have no geometry of their own, so in a bare file they only show as joint gizmos (enable
Show > Physics > Joints); stack_sim's simulator draws them. To fly the rig, use
[stack_sim](../stack_sim/) (`make sim-pegasus-ros2`).

## Configuring the rig

Everything is a key in `payload_rig.yaml`, overridable with `--set key=value` on either script:

| Want | Key |
|---|---|
| number of drones (>= 1) | `num_drones` |
| payload size / mass | `payload.radius`, `payload.height`, `payload.mass` (`payload.inertia` to override the solid cylinder) |
| where cables attach on the payload | `payload.anchor_radius` (default: the rim), `payload.anchor_height` (default: the top face) |
| how far out the drones sit | `formation.horizontal_distance` (payload axis to each cable mount), or `formation.cable_angle_deg` |
| cable length | `cable.length` |
| cable model | `cable.model: distance` (PhysX distance joint, can go slack) or `rope` (rigid segments + ball joints) |
| mount height on the landing gear | `mount.height`: 0 = top of the poles, just under the frame; `landing_gear.pole_length` = bottom of the poles; 0.163 = level with the skids, the lowest |
| release mechanisms | `release.location: none / drone / payload / both`, `release.actuation_delay` |

The layout check rejects formations whose neighbouring rotor discs come closer than
`formation.min_rotor_clearance` (with the defaults, N = 6 needs `formation.horizontal_distance` of at least 0.82 m).

## Modelling notes and measured facts

- **Joints between Pegasus vehicles work.** Each drone stays its own articulation; cable joints are authored with
  `physics:excludeFromArticulation`, so PhysX solves them as maximal-coordinate constraints. The rope model makes each
  rope its own articulation (segments joined inside it) with maximal joints only at its two ends. Nothing has to be
  merged into one articulation.
- **PhysX distance joints have a 25 mm dead band.** A distance joint only engages 25 mm past `maxDistance` (measured
  with 1 m and 2 m cables; the PhysX tolerance is not exposed in USD), so the builder sets
  `maxDistance = length - 0.025`. Without that, every cable is 2.5 cm too long.
- **Sleeping is off** for the payload, rope segments and drones. A payload hanging still falls asleep, and switching
  off a joint does not wake it, so a release would silently do nothing.
- **Release** switches the joint off (`physics:jointEnabled = False`); PhysX applies it on the next physics step.
  There is no latch dynamics; the servo is modelled only by `release.actuation_delay`.
- **Tension** is computed from the payload's Newton-Euler equations (exact for massless cables while the payload is
  airborne). The spring-formula value logged for the distance model reads 2-3 % low and is only a cross-check.
- The S500's mass budget (1.500 kg all-up) and several dimensions are estimates from photos; the YAML marks which.
  Measure the real airframe and put the numbers in `s500.yaml`.

## Training asset (flycrane)

```bash
make sim-up
docker compose -f docker/docker-compose.sim.yml exec -w /home/ws/components/simulation/assets simulation \
  /isaac-sim/python.sh build_assets.py --flycrane --set num_drones=3 --set payload.mass=1.4
```

writes `generated/flycrane_s500_n<N>/`: `flycrane.urdf`, `flycrane.usd` (Isaac Lab's URDF converter, fixed joints
kept) and `params.json` (layout, masses, sources, and `drone_params`). The MARL ext's flycrane envs look their bodies
up by name (`Falcon<k>/base_link_inertia`, `Falcon<k>/rotor_<j>`, `rope_<k>_link`, `load/odometry_sensor_link`), so
the URDF is cloned from the ext's one-rod `flycrane_rod` template, names and joint structure unchanged (for N = 3 the
set of links and joints is identical), with every hardware number replaced from `s500.yaml` and the rig: S500 body
mass, CoM and inertia, rotor positions in the Falcon's rotor order, the payload cylinder, anchors, cable length, mass
and damping, the cable angle and drone yaw. All joints at zero is the spawn formation, drones level.

Not done by this: the ext's controllers (`controllers/geometric.py`, `indi.py`, `motor_model.py`) and the env cfg's
`max_thrust_pp` hardcode the Falcon (0.6017 kg, arm 0.106 m, 6.25 N per rotor, its thrust map and gains).
`drone_params` in `params.json` has the S500 values in the same terms; wiring them in, and retuning the gains for an
S500, is a change to the training setup.

The template's `base_link` has no `<inertial>`, and the URDF importer gives such a link 1 kg. In the shipped
`flycrane_rod.usd` every Falcon is therefore 1.617 kg in PhysX, not 0.6017 kg (measured in Isaac Sim 5.1); the
generated asset gives `base_link` a sensor-sized mass instead.

