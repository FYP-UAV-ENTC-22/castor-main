"""The rig a policy flies, from the assets: which rig file, whether it is the rig the model was trained on, how high
PX4 takes off, and every PX4 SITL setting. Plain Python + numpy + PyYAML, so it runs on the host (stack_sim.sh) and in
Isaac Sim's Python (stack_sim_pegasus.py) alike.

    python3 components/simulation/stack_sim/rig_check.py [--model NAME/VERSION|DIR] [--rig FILE] [--shell]

Sources of truth:
  - the model package (models/<name>/<version>/model.yaml) names the rig it was trained on (rig.config); its other rig
    numbers are checked against that file and a mismatch is an error, because the onboard runner flies with them;
  - components/simulation/assets/config: the rig file, its vehicle file, and px4_sitl.yaml for PX4's SIL settings;
  - take-off height: the height above home where the cables go taut (from the rig geometry) minus px4_sitl.yaml's
    takeoff.taut_margin is the floor; the model's rig.takeoff_height is used when it is above that floor.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass, field

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CASTOR_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
MODELS_ROOT = os.path.join(CASTOR_ROOT, "models")
sys.path.insert(0, os.path.join(CASTOR_ROOT, "components", "simulation", "assets"))

from castor_assets import config as C  # noqa: E402
from castor_assets import geometry as G  # noqa: E402

PX4_SITL = "px4_sitl.yaml"
TOLERANCE_M = 1e-3
TOLERANCE_DEG = 0.1


class RigError(ValueError):
    pass


def resolve_model(spec: str | None = None, models_root: str = MODELS_ROOT) -> tuple[str, str]:
    """(id, directory) of a model package: <name>/<version> under models/, a package directory, or models/DEFAULT."""
    if not spec:
        with open(os.path.join(models_root, "DEFAULT")) as f:
            spec = f.read().strip()
    if os.path.isfile(os.path.join(models_root, spec, "model.yaml")):
        return spec, os.path.join(models_root, spec)
    if os.path.isfile(os.path.join(spec, "model.yaml")):
        path = os.path.realpath(spec)
        return f"stack_sim/{os.path.basename(path)}", path
    raise RigError(f"model '{spec}' not found: give <name>/<version> under {models_root} or a package directory")


@dataclass
class RigPlan:
    model_id: str
    model_dir: str
    rig_file: str
    rig: C.RigCfg
    vehicle: C.VehicleCfg
    sitl: dict
    mount_local: list
    anchors_local: list
    drone_yaw_deg: list
    home_height: float          # body origin above the ground, standing on the skids
    taut_height: float          # m above home where every cable is just taut (drones straight above their spot)
    takeoff_floor: float        # taut_height - margin
    takeoff_config: float | None
    takeoff_height: float       # what PX4 is asked for
    mismatches: list = field(default_factory=list)
    notes: list = field(default_factory=list)


def _close(a, b, tol):
    return len(a) == len(b) and all(abs(float(x) - float(y)) <= tol for x, y in zip(a, b))


def plan(model: str | None = None, rig_override: str | None = None, rig_overrides=(), vehicle_overrides=(),
         takeoff_override: float | None = None) -> RigPlan:
    model_id, model_dir = resolve_model(model)
    with open(os.path.join(model_dir, "model.yaml")) as f:
        manifest = yaml.safe_load(f) or {}
    mrig = manifest.get("rig") or {}
    rig_file = rig_override or mrig.get("config")
    if not rig_file:
        raise RigError(f"{model_id}: model.yaml names no rig (rig.config) and no --rig was given")
    rig = C.load_rig(rig_file, list(rig_overrides))
    vehicle = C.load_vehicle(rig.vehicle_config, list(vehicle_overrides))
    sitl = C.load_yaml(PX4_SITL)
    if rig.vehicle != "s500":
        raise RigError(f"{rig_file}: vehicle {rig.vehicle!r}; stack_sim derives PX4 and the take-off from the S500 "
                       f"vehicle file only")

    geo = G.S500Geometry(vehicle)
    mount = geo.mount_point(rig.mount.height)
    mass = G.vehicle_mass_properties(vehicle).total_mass
    layout = G.rig_layout(rig, mount, geo.rotor_envelope_radius, mass, 4 * vehicle.propulsion.max_thrust_per_motor)
    anchors = [[float(x) for x in a] for a in layout.anchors_local]
    yaw_deg = [math.degrees(y) for y in layout.drone_yaw]

    mismatches = []
    if mrig.get("config") and rig_override and os.path.basename(rig_override) != mrig["config"]:
        mismatches.append(f"rig file {rig_override}, but the model was trained on {mrig['config']}")
    checks = [
        ("cable_length", [mrig.get("cable_length")], [rig.cable.length], TOLERANCE_M),
        ("payload_height", [mrig.get("payload_height")], [rig.payload.height], TOLERANCE_M),
        ("mount_local", mrig.get("mount_local"), list(mount), TOLERANCE_M),
        ("anchors_local", sum(mrig.get("anchors_local") or [], []), sum(anchors, []), TOLERANCE_M),
        ("drone_yaw_deg", mrig.get("drone_yaw_deg"), yaw_deg, TOLERANCE_DEG),
    ]
    for key, model_value, asset_value, tol in checks:
        if model_value is None or (isinstance(model_value, list) and None in model_value):
            continue  # the manifest does not give it
        if not _close(model_value, asset_value, tol):
            mismatches.append(f"rig.{key}: model {model_value}, assets {[round(v, 4) for v in asset_value]}")
    if len(mrig.get("anchors_local") or []) not in (0, rig.num_drones):
        mismatches.append(f"model has {len(mrig['anchors_local'])} cables, the rig {rig.num_drones} drones")

    # The cables go taut when the mount, straight above where the drone stood, is one cable length from its anchor on
    # the resting payload. Heights from the ground; PX4's take-off height is above home (the body origin on the skids).
    home = geo.spawn_height_on_ground
    anchor_z = rig.payload.height / 2.0 + anchors[0][2]
    reach = rig.cable.length ** 2 - (layout.horizontal_distance - layout.anchor_radius) ** 2
    taut = anchor_z + math.sqrt(max(reach, 0.0)) - mount[2] - home
    margin = float((sitl.get("takeoff") or {}).get("taut_margin", 0.3))
    floor = taut - margin
    config = takeoff_override if takeoff_override is not None else mrig.get("takeoff_height")
    notes = []
    if config is None:
        height = floor
        notes.append(f"no take-off height configured: {height:.2f} m (cables taut at {taut:.2f} m, margin {margin} m)")
    elif config < floor:
        height = floor
        notes.append(f"configured take-off height {config:.2f} m is below the floor {floor:.2f} m (cables taut at "
                     f"{taut:.2f} m above home, margin {margin} m): taking off to {height:.2f} m")
    else:
        height = float(config)
        notes.append(f"take-off height {height:.2f} m as configured (floor {floor:.2f} m, cables taut at {taut:.2f} m)"
                     + ("; above the taut height, PX4's take-off lifts the payload" if height > taut else ""))
    return RigPlan(model_id, model_dir, rig_file, rig, vehicle, sitl, [float(x) for x in mount], anchors, yaw_deg, home,
                   taut, floor, config, height, mismatches, notes)


def px4_parameters(p: RigPlan, build: str, physics_hz: float, drone_mass: float | None = None,
                   payload: bool = True) -> dict:
    """PX4_PARAM_* for every SITL instance: px4_sitl.yaml plus what the vehicle file decides."""
    params = dict(p.sitl.get("params") or {})
    if build == "px4_sitl_raptor":
        params.update(p.sitl.get("raptor_params") or {})
        params["IMU_GYRO_RATEMAX"] = int(physics_hz)  # mc_raptor leaves its mode on gyro samples older than 10 ms
    gps = p.sitl.get("gps") or {}
    if gps.get("model") == "rtk":
        params.update(gps.get("rtk_params") or {})
    geo = G.S500Geometry(p.vehicle)
    prop = p.vehicle.propulsion
    # PX4 quad-X order matches the vehicle file's rotor order; PX4's frame is FRD (y to the right), the body's FLU.
    # KM: yaw moment per thrust, positive for a counter-clockwise rotor (spin_directions: -1 = CCW, Pegasus' sign).
    for i, ((x, y), spin) in enumerate(zip(geo.rotor_xy, prop.spin_directions)):
        params.update({f"CA_ROTOR{i}_PX": round(x, 4), f"CA_ROTOR{i}_PY": round(-y, 4),
                       f"CA_ROTOR{i}_KM": round(prop.torque_to_thrust * (1.0 if spin < 0 else -1.0), 4)})
    mass = drone_mass if drone_mass is not None else G.vehicle_mass_properties(p.vehicle).total_mass
    share = p.rig.payload.mass / p.rig.num_drones if payload else 0.0
    hover = (mass + share) * G.GRAVITY / (4 * prop.max_thrust_per_motor)
    # Thrust goes with rotor speed squared, and the simulated motors map a command u to (max - 100) u + 100 rad/s.
    w_max = prop.max_rotor_velocity
    params["MPC_THR_HOVER"] = round((w_max * math.sqrt(hover) - 100.0) / (w_max - 100.0), 3)
    return {f"PX4_PARAM_{k}": str(v) for k, v in params.items()}


def gps_sensor(p: RigPlan) -> dict | None:
    """Pegasus GPS sensor settings, or None for Pegasus' default receiver."""
    gps = p.sitl.get("gps") or {}
    return dict(gps["rtk_sensor"]) if gps.get("model") == "rtk" else None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--model", default=None, help="<name>/<version> under models/ or a package dir; default DEFAULT")
    ap.add_argument("--rig", default=None, help="another rig file (must match the model's rig)")
    ap.add_argument("--takeoff-height", type=float, default=None, help="instead of the model's rig.takeoff_height")
    ap.add_argument("--shell", action="store_true", help="KEY=value lines for a shell; notes go to stderr")
    a = ap.parse_args()
    try:
        p = plan(a.model, a.rig, takeoff_override=a.takeoff_height)
    except (RigError, C.ConfigError, OSError) as e:
        print(f"rig_check: {e}", file=sys.stderr)
        sys.exit(2)
    out = sys.stderr if a.shell else sys.stdout
    print(f"model {p.model_id}: rig {p.rig_file}, {p.rig.num_drones} x {p.rig.vehicle}, cables {p.rig.cable.length} m",
          file=out)
    for n in p.notes:
        print(f"  {n}", file=out)
    for m in p.mismatches:
        print(f"  MISMATCH {m}", file=out)
    if a.shell:
        print(f"MODEL_ID={p.model_id}\nMODEL_DIR={p.model_dir}\nRIG={p.rig_file}\nDRONES={p.rig.num_drones}\n"
              f"TAKEOFF_HEIGHT={p.takeoff_height:.2f}\nTAUT_HEIGHT={p.taut_height:.2f}")
    if p.mismatches:
        print("rig_check: the rig in the assets is not the rig this model was trained on (see MISMATCH)",
              file=sys.stderr)
        sys.exit(3)


if __name__ == "__main__":
    main()
