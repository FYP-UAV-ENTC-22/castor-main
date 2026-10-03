"""Load and validate robot.yaml, the one file that gives a host its robot identity.

Every container mounts the same file (default /etc/castor/robot.yaml, override
with CASTOR_ROBOT_CONFIG). Changing it and restarting the stack changes the ROS
namespace, the policy's one-hot agent id and the device paths everywhere at once.

Validation is strict on purpose: unknown keys are errors, so a typo such as
`namspace:` fails loudly at container start instead of silently falling back
to a default.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from typing import Any

import yaml

DEFAULT_PATH = "/etc/castor/robot.yaml"
ENV_PATH = "CASTOR_ROBOT_CONFIG"

HARDWARE = ("rpi4", "rpi5", "laptop", "workstation")
FC_TRANSPORTS = ("serial", "udp")
ZENOH_ROLES = ("drone", "ground_station")

_ROS_NAME = re.compile(r"^[a-z][a-z0-9_]*$")
_ZENOH_ENDPOINT = re.compile(r"^(tcp|udp|tls|quic)/[^\s/]+:\d{1,5}$")
_UDP_TARGET = re.compile(r"^[^\s:]+:\d{1,5}$")


class ConfigError(ValueError):
    """robot.yaml is missing or invalid. The message lists every problem found."""


@dataclass(frozen=True)
class FcConfig:
    enabled: bool = False
    transport: str = "serial"        # serial (real FC) | udp (PX4 SITL)
    device: str = "/dev/ttyAMA0"
    baud: int = 921600
    udp_port: int = 8888
    px4_namespace: str = ""          # uxrce_dds_client namespace; "" -> /fmu/...


@dataclass(frozen=True)
class MavlinkConfig:
    enabled: bool = False            # mavlink-router: FC MAVLink port -> UDP for QGC
    device: str = ""
    baud: int = 921600
    gcs_endpoints: tuple[str, ...] = ("127.0.0.1:14550",)


@dataclass(frozen=True)
class UwbConfig:
    enabled: bool = False
    device: str = "/dev/ttyACM0"


@dataclass(frozen=True)
class ZenohConfig:
    role: str = "drone"              # drone | ground_station: decides the bridge allow-lists
    connect: tuple[str, ...] = ()    # other bridges / the ground station, e.g. tcp/10.0.0.5:7447
    listen_port: int = 7447
    multicast_scouting: bool = True  # find other bridges on the same network automatically


@dataclass(frozen=True)
class RobotConfig:
    robot_id: int
    namespace: str
    hardware: str
    team_size: int
    team_index: int
    fc: FcConfig = field(default_factory=FcConfig)
    mavlink: MavlinkConfig = field(default_factory=MavlinkConfig)
    uwb: UwbConfig = field(default_factory=UwbConfig)
    zenoh: ZenohConfig = field(default_factory=ZenohConfig)
    source: str = ""

    def one_hot(self) -> list[float]:
        """Agent-id vector for the policy observation: 1.0 at team_index."""
        return [1.0 if i == self.team_index else 0.0 for i in range(self.team_size)]

    def env(self) -> dict[str, str]:
        """The subset exported to the shell by the component entrypoints."""
        return {
            "CASTOR_ROBOT_ID": str(self.robot_id),
            "CASTOR_NS": self.namespace,
            "CASTOR_HARDWARE": self.hardware,
            "CASTOR_TEAM_SIZE": str(self.team_size),
            "CASTOR_TEAM_INDEX": str(self.team_index),
        }


# section -> {key: (type, required)}
_SCHEMA: dict[str, dict[str, tuple[type, bool]]] = {
    "robot": {"id": (int, True), "namespace": (str, True), "hardware": (str, True)},
    "team": {"size": (int, True), "index": (int, True)},
    "fc": {
        "enabled": (bool, False), "transport": (str, False), "device": (str, False),
        "baud": (int, False), "udp_port": (int, False), "px4_namespace": (str, False),
    },
    "mavlink": {
        "enabled": (bool, False), "device": (str, False), "baud": (int, False),
        "gcs_endpoints": (list, False),
    },
    "uwb": {"enabled": (bool, False), "device": (str, False)},
    "zenoh": {
        "role": (str, False), "connect": (list, False), "listen_port": (int, False),
        "multicast_scouting": (bool, False),
    },
}
_REQUIRED_SECTIONS = ("robot", "team")


def _type_name(t: type) -> str:
    return {int: "an integer", str: "a string", bool: "true/false", list: "a list"}[t]


def _check_types(data: dict[str, Any], errors: list[str]) -> None:
    for section in data:
        if section not in _SCHEMA:
            errors.append(f"unknown section '{section}' (allowed: {', '.join(_SCHEMA)})")
    for section in _REQUIRED_SECTIONS:
        if section not in data:
            errors.append(f"missing required section '{section}'")
    for section, keys in _SCHEMA.items():
        value = data.get(section)
        if value is None:
            continue
        if not isinstance(value, dict):
            errors.append(f"'{section}' must be a mapping")
            continue
        for key in value:
            if key not in keys:
                errors.append(f"unknown key '{section}.{key}' (allowed: {', '.join(keys)})")
        for key, (typ, required) in keys.items():
            if key not in value:
                if required:
                    errors.append(f"missing required key '{section}.{key}'")
                continue
            v = value[key]
            # bool is a subclass of int; don't let `id: true` pass as an integer.
            if typ is int and isinstance(v, bool) or not isinstance(v, typ):
                errors.append(f"'{section}.{key}' must be {_type_name(typ)}, got {v!r}")


def parse(data: Any, source: str = "<string>") -> RobotConfig:
    """Validate an already-loaded YAML document and build a RobotConfig."""
    if not isinstance(data, dict):
        raise ConfigError(f"{source}: top level must be a mapping")

    errors: list[str] = []
    _check_types(data, errors)
    if errors:
        raise ConfigError(f"{source} is invalid:\n  - " + "\n  - ".join(errors))

    robot, team = data["robot"], data["team"]
    fc_d, mav_d = data.get("fc") or {}, data.get("mavlink") or {}
    uwb_d, zen_d = data.get("uwb") or {}, data.get("zenoh") or {}

    if robot["id"] < 1:
        errors.append(f"'robot.id' must be >= 1, got {robot['id']}")
    if not _ROS_NAME.match(robot["namespace"]):
        errors.append(
            f"'robot.namespace' must be lowercase letters, digits and '_', starting with a letter; "
            f"got {robot['namespace']!r}")
    if robot["hardware"] not in HARDWARE:
        errors.append(f"'robot.hardware' must be one of {', '.join(HARDWARE)}; got {robot['hardware']!r}")
    if team["size"] < 1:
        errors.append(f"'team.size' must be >= 1, got {team['size']}")
    elif not 0 <= team["index"] < team["size"]:
        errors.append(f"'team.index' must be in 0..{team['size'] - 1} for team.size {team['size']}, got {team['index']}")

    fc = FcConfig(**fc_d)
    if fc.transport not in FC_TRANSPORTS:
        errors.append(f"'fc.transport' must be one of {', '.join(FC_TRANSPORTS)}; got {fc.transport!r}")
    if fc.px4_namespace and not _ROS_NAME.match(fc.px4_namespace):
        errors.append(f"'fc.px4_namespace' must be empty or a ROS name token; got {fc.px4_namespace!r}")
    if fc.enabled and fc.transport == "serial" and not fc.device.startswith("/dev/"):
        errors.append(f"'fc.device' must be a /dev path when fc.transport is serial; got {fc.device!r}")

    endpoints = tuple(mav_d.pop("gcs_endpoints", MavlinkConfig.gcs_endpoints))
    for ep in endpoints:
        if not isinstance(ep, str) or not _UDP_TARGET.match(ep):
            errors.append(f"'mavlink.gcs_endpoints' entries must look like 'host:port'; got {ep!r}")
    mavlink = MavlinkConfig(gcs_endpoints=endpoints, **mav_d)
    if mavlink.enabled and not mavlink.device.startswith("/dev/"):
        errors.append(f"'mavlink.device' must be a /dev path when mavlink.enabled; got {mavlink.device!r}")

    uwb = UwbConfig(**uwb_d)

    connect = tuple(zen_d.pop("connect", ()))
    for ep in connect:
        if not isinstance(ep, str) or not _ZENOH_ENDPOINT.match(ep):
            errors.append(f"'zenoh.connect' entries must look like 'tcp/<host>:<port>'; got {ep!r}")
    zenoh = ZenohConfig(connect=connect, **zen_d)
    if zenoh.role not in ZENOH_ROLES:
        errors.append(f"'zenoh.role' must be one of {', '.join(ZENOH_ROLES)}; got {zenoh.role!r}")

    for name, port in (("fc.udp_port", fc.udp_port), ("zenoh.listen_port", zenoh.listen_port)):
        if not 1 <= port <= 65535:
            errors.append(f"'{name}' must be a port number, got {port}")

    if errors:
        raise ConfigError(f"{source} is invalid:\n  - " + "\n  - ".join(errors))

    return RobotConfig(
        robot_id=robot["id"], namespace=robot["namespace"], hardware=robot["hardware"],
        team_size=team["size"], team_index=team["index"],
        fc=fc, mavlink=mavlink, uwb=uwb, zenoh=zenoh, source=source,
    )


def config_path(path: str | None = None) -> str:
    return path or os.environ.get(ENV_PATH) or DEFAULT_PATH


def load(path: str | None = None) -> RobotConfig:
    """Read and validate robot.yaml. Raises ConfigError with a readable message."""
    path = config_path(path)
    if os.path.isdir(path):
        raise ConfigError(
            f"{path} is a directory, not a file. The host file was missing when the container "
            f"started, so Docker created an empty directory in its place. Remove that directory "
            f"on the host and create the file from deploy/robot.example.yaml.")
    if not os.path.exists(path):
        raise ConfigError(
            f"robot config not found at {path}. Create it from deploy/robot.example.yaml, "
            f"or point {ENV_PATH} at another file.")
    try:
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise ConfigError(f"{path} is not valid YAML: {e}") from e
    return parse(data, source=path)
