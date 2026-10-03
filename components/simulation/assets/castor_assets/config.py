"""Typed configuration for the S500 vehicle and the payload rig, loaded from YAML.

Unknown keys are an error: a typo in a YAML file must not silently fall back to a default.
Overrides use dotted paths, e.g. ``cable.length=1.2`` or ``num_drones=4``; values are parsed as YAML scalars.
"""

from __future__ import annotations

import copy
import dataclasses
import os
import typing
from dataclasses import dataclass, field
from typing import Optional

import yaml

from . import ASSETS_ROOT

CONFIG_DIR = os.path.join(ASSETS_ROOT, "config")
GENERATED_DIR = os.path.join(ASSETS_ROOT, "generated")


class ConfigError(ValueError):
    pass


# --------------------------------------------------------------------------------------------------------------------
# Vehicle (S500)
# --------------------------------------------------------------------------------------------------------------------


@dataclass
class FrameCfg:
    wheelbase: float = 0.480
    plate_size: list = field(default_factory=lambda: [0.150, 0.150])
    plate_thickness: float = 0.0015
    arm_width: float = 0.034
    arm_height: float = 0.022
    arm_root_radius: float = 0.055


@dataclass
class MotorCfg:
    diameter: float = 0.028
    height: float = 0.030


@dataclass
class PropellerCfg:
    diameter: float = 0.2388
    mass: float = 0.014
    height_above_motor: float = 0.008
    show_disc: bool = False


@dataclass
class LandingGearCfg:
    pole_diameter: float = 0.016
    pole_length: float = 0.140
    pole_spacing: float = 0.120
    pole_x: float = 0.0
    top_mount_height: float = 0.012
    tee_size: list = field(default_factory=lambda: [0.032, 0.026, 0.036])
    skid_diameter: float = 0.016
    foam_diameter: float = 0.026
    skid_length: float = 0.280


@dataclass
class BatteryMountCfg:
    enabled: bool = True
    rail_diameter: float = 0.010
    rail_length: float = 0.300
    rail_spacing: float = 0.068
    drop: float = 0.023


@dataclass
class MassItemCfg:
    name: str = ""
    mass: float = 0.0
    kind: str = "box"  # box | motors | arms | landing_gear
    size: Optional[list] = None  # box only
    pos: Optional[list] = None  # box only, body frame


@dataclass
class MassCfg:
    total: Optional[float] = None
    items: list = field(default_factory=list)  # list[MassItemCfg]
    com_override: Optional[list] = None
    inertia_override: Optional[list] = None


@dataclass
class PropulsionCfg:
    max_thrust_per_motor: float = 11.5
    torque_to_thrust: float = 0.0138
    max_rotor_velocity: float = 1100.0
    spin_directions: list = field(default_factory=lambda: [-1, -1, 1, 1])
    motor_time_constant: float = 0.0
    linear_drag: list = field(default_factory=lambda: [0.50, 0.30, 0.0])


@dataclass
class VehiclePhysicsCfg:
    solver_position_iterations: int = 32
    solver_velocity_iterations: int = 1
    angular_damping: float = 0.05


@dataclass
class VehicleCfg:
    name: str = "s500"
    frame: FrameCfg = field(default_factory=FrameCfg)
    motor: MotorCfg = field(default_factory=MotorCfg)
    propeller: PropellerCfg = field(default_factory=PropellerCfg)
    landing_gear: LandingGearCfg = field(default_factory=LandingGearCfg)
    battery_mount: BatteryMountCfg = field(default_factory=BatteryMountCfg)
    mass: MassCfg = field(default_factory=MassCfg)
    propulsion: PropulsionCfg = field(default_factory=PropulsionCfg)
    physics: VehiclePhysicsCfg = field(default_factory=VehiclePhysicsCfg)


# --------------------------------------------------------------------------------------------------------------------
# Payload rig
# --------------------------------------------------------------------------------------------------------------------


@dataclass
class PayloadCfg:
    radius: float = 0.25
    height: float = 0.16
    mass: float = 1.4
    com_offset: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    inertia: Optional[list] = None
    anchor_radius: Optional[float] = None
    anchor_height: Optional[float] = None
    angle_offset_deg: float = 0.0


@dataclass
class FormationCfg:
    horizontal_distance: Optional[float] = None
    cable_angle_deg: Optional[float] = 28.65
    drone_yaw: str = "zero"  # zero | outward
    min_rotor_clearance: float = 0.10


@dataclass
class MountCfg:
    height: float = 0.07
    rod_diameter: float = 0.010


@dataclass
class CableCfg:
    model: str = "distance"  # distance | rope
    length: float = 1.0
    radius: float = 0.0025
    stiffness: Optional[float] = 20000.0
    damping: float = 50.0
    segments: int = 7
    mass: float = 0.0236
    joint_damping: float = 0.005
    flycrane_inertia: bool = True
    collisions: bool = False


@dataclass
class ReleaseCfg:
    location: str = "both"  # none | drone | payload | both
    actuation_delay: float = 0.0


@dataclass
class SpawnCfg:
    payload_clearance: float = 0.5  # payload bottom above the ground at spawn
    position: list = field(default_factory=lambda: [0.0, 0.0])  # payload axis, x y


@dataclass
class RigPhysicsCfg:
    solver_position_iterations: int = 32
    solver_velocity_iterations: int = 1


@dataclass
class RigCfg:
    num_drones: int = 3
    vehicle: str = "s500"  # s500 | iris
    vehicle_config: str = "s500.yaml"
    payload: PayloadCfg = field(default_factory=PayloadCfg)
    formation: FormationCfg = field(default_factory=FormationCfg)
    mount: MountCfg = field(default_factory=MountCfg)
    cable: CableCfg = field(default_factory=CableCfg)
    release: ReleaseCfg = field(default_factory=ReleaseCfg)
    spawn: SpawnCfg = field(default_factory=SpawnCfg)
    physics: RigPhysicsCfg = field(default_factory=RigPhysicsCfg)


# --------------------------------------------------------------------------------------------------------------------
# Loading
# --------------------------------------------------------------------------------------------------------------------


def _build(cls, data, path):
    """Recursively turn a dict into dataclass `cls`, rejecting unknown keys."""
    if data is None:
        return cls()
    if not isinstance(data, dict):
        raise ConfigError(f"{path or '<root>'}: expected a mapping, got {type(data).__name__}")
    hints = typing.get_type_hints(cls)
    names = {f.name for f in dataclasses.fields(cls)}
    unknown = sorted(set(data) - names)
    if unknown:
        raise ConfigError(f"{path or '<root>'}: unknown key(s) {unknown}; valid: {sorted(names)}")
    kwargs = {}
    for f in dataclasses.fields(cls):
        if f.name not in data:
            continue
        value = data[f.name]
        hint = hints[f.name]
        sub = f"{path}.{f.name}" if path else f.name
        if dataclasses.is_dataclass(hint):
            kwargs[f.name] = _build(hint, value, sub)
        elif cls is MassCfg and f.name == "items":
            kwargs[f.name] = [_build(MassItemCfg, v, f"{sub}[{i}]") for i, v in enumerate(value or [])]
        else:
            kwargs[f.name] = value
    return cls(**kwargs)


def apply_overrides(data: dict, overrides) -> dict:
    """Apply ``a.b.c=value`` strings to a nested dict (value parsed as YAML)."""
    data = copy.deepcopy(data)
    for item in overrides or []:
        if "=" not in item:
            raise ConfigError(f"override {item!r} is not key=value")
        key, raw = item.split("=", 1)
        parts = key.strip().split(".")
        node = data
        for p in parts[:-1]:
            node = node.setdefault(p, {})
            if not isinstance(node, dict):
                raise ConfigError(f"override {item!r}: {p} is not a mapping")
        node[parts[-1]] = yaml.safe_load(raw)
    return data


def resolve_config_path(path: str) -> str:
    if os.path.isabs(path) or os.path.exists(path):
        return os.path.abspath(path)
    return os.path.join(CONFIG_DIR, path)


def load_yaml(path: str) -> dict:
    with open(resolve_config_path(path)) as f:
        return yaml.safe_load(f) or {}


def load_vehicle(path: str = "s500.yaml", overrides=None) -> VehicleCfg:
    cfg = _build(VehicleCfg, apply_overrides(load_yaml(path), overrides), "")
    validate_vehicle(cfg)
    return cfg


def load_rig(path: str = "payload_rig.yaml", overrides=None) -> RigCfg:
    cfg = _build(RigCfg, apply_overrides(load_yaml(path), overrides), "")
    validate_rig(cfg)
    return cfg


def to_dict(cfg) -> dict:
    return dataclasses.asdict(cfg)


# --------------------------------------------------------------------------------------------------------------------
# Validation (geometry-dependent checks live in geometry.check_layout)
# --------------------------------------------------------------------------------------------------------------------


def _positive(name, value):
    if value is None or value <= 0:
        raise ConfigError(f"{name} must be > 0, got {value}")


def validate_vehicle(v: VehicleCfg):
    for name in ("wheelbase", "plate_thickness", "arm_width", "arm_height", "arm_root_radius"):
        _positive(f"frame.{name}", getattr(v.frame, name))
    for name in ("pole_diameter", "pole_length", "pole_spacing", "skid_diameter", "foam_diameter", "skid_length"):
        _positive(f"landing_gear.{name}", getattr(v.landing_gear, name))
    _positive("propeller.diameter", v.propeller.diameter)
    _positive("propeller.mass", v.propeller.mass)
    _positive("propulsion.max_thrust_per_motor", v.propulsion.max_thrust_per_motor)
    _positive("propulsion.max_rotor_velocity", v.propulsion.max_rotor_velocity)
    if v.propulsion.torque_to_thrust < 0:
        raise ConfigError("propulsion.torque_to_thrust must be >= 0")
    if sorted(abs(s) for s in v.propulsion.spin_directions) != [1, 1, 1, 1]:
        raise ConfigError("propulsion.spin_directions must be four values of +-1")
    if v.propulsion.motor_time_constant < 0:
        raise ConfigError("propulsion.motor_time_constant must be >= 0")
    if not v.mass.items:
        raise ConfigError("mass.items is empty")
    for i, item in enumerate(v.mass.items):
        if item.kind not in ("box", "motors", "arms", "landing_gear"):
            raise ConfigError(f"mass.items[{i}].kind {item.kind!r} not in box|motors|arms|landing_gear")
        if item.mass < 0:
            raise ConfigError(f"mass.items[{i}].mass must be >= 0")
        if item.kind == "box" and (item.size is None or item.pos is None):
            raise ConfigError(f"mass.items[{i}] ({item.name}): a box needs size and pos")
    if v.mass.total is not None:
        _positive("mass.total", v.mass.total)
        if v.mass.total <= 4 * v.propeller.mass:
            raise ConfigError("mass.total must exceed the four propellers' mass")
    if v.mass.inertia_override is not None and (len(v.mass.inertia_override) != 3 or min(v.mass.inertia_override) <= 0):
        raise ConfigError("mass.inertia_override must be three positive numbers [ixx, iyy, izz]")


def validate_rig(r: RigCfg):
    if not isinstance(r.num_drones, int) or r.num_drones < 1:
        raise ConfigError(f"num_drones must be an integer >= 1, got {r.num_drones!r}")
    if r.vehicle not in ("s500", "iris"):
        raise ConfigError(f"vehicle must be s500 or iris, got {r.vehicle!r}")
    for name in ("radius", "height", "mass"):
        _positive(f"payload.{name}", getattr(r.payload, name))
    if r.payload.anchor_radius is not None and not 0 <= r.payload.anchor_radius <= r.payload.radius + 1e-9:
        raise ConfigError("payload.anchor_radius must be between 0 and payload.radius")
    if r.payload.inertia is not None and (len(r.payload.inertia) != 3 or min(r.payload.inertia) <= 0):
        raise ConfigError("payload.inertia must be three positive numbers [ixx, iyy, izz]")
    f = r.formation
    if f.horizontal_distance is None and f.cable_angle_deg is None:
        raise ConfigError("set formation.horizontal_distance or formation.cable_angle_deg")
    if f.horizontal_distance is not None and f.horizontal_distance < 0:
        raise ConfigError("formation.horizontal_distance must be >= 0")
    if f.cable_angle_deg is not None and not -89.0 < f.cable_angle_deg < 89.0:
        raise ConfigError("formation.cable_angle_deg must be within (-89, 89)")
    if f.drone_yaw not in ("zero", "outward"):
        raise ConfigError("formation.drone_yaw must be zero or outward")
    if r.mount.height < 0:
        raise ConfigError("mount.height must be >= 0 (0 = top of the landing-gear poles)")
    _positive("mount.rod_diameter", r.mount.rod_diameter)
    c = r.cable
    if c.model not in ("distance", "rope"):
        raise ConfigError(f"cable.model must be distance or rope, got {c.model!r}")
    _positive("cable.length", c.length)
    _positive("cable.radius", c.radius)
    if c.stiffness is not None:
        _positive("cable.stiffness", c.stiffness)
    if c.damping < 0:
        raise ConfigError("cable.damping must be >= 0")
    if not isinstance(c.segments, int) or c.segments < 1:
        raise ConfigError("cable.segments must be an integer >= 1")
    if c.model == "rope":
        _positive("cable.mass", c.mass)
    if r.release.location not in ("none", "drone", "payload", "both"):
        raise ConfigError("release.location must be none, drone, payload or both")
    if r.release.actuation_delay < 0:
        raise ConfigError("release.actuation_delay must be >= 0")
    if r.spawn.payload_clearance < 0:
        raise ConfigError("spawn.payload_clearance must be >= 0")
    _positive("physics.solver_position_iterations", r.physics.solver_position_iterations)
