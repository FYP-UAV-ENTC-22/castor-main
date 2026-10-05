"""Render the zenoh-bridge-ros2dds configuration for one host.

Inside a host every container talks Fast DDS over loopback and shared memory
only (docker/fastdds_localhost.xml). The bridge is the single way traffic leaves
or enters the host, and it carries only what the allow-lists below name. Its own
Cyclone DDS is pinned to 127.0.0.1 by CYCLONEDDS_URI in the compose file. Raw PX4 topics (/fmu/...) are never bridged: their type
hashes and QoS are known not to route through it, and nobody off-board should
be talking to the flight controller directly anyway.

The output is plain JSON, which is valid JSON5.
"""

from __future__ import annotations

import json
import os

from .robot_config import RobotConfig

# How often odometry may cross the radio link; full rate stays on the host.
ODOM_MAX_HZ = 20


# What a drone shares about itself: liveness, vehicle state, supervisor state, mission state, policy status.
_DRONE_TOPICS = [
    "[a-z_]+/heartbeat",
    "vehicle/(odom|state)",
    "system/state",
    "system/mission",
    "planning/status",
]
_ANY_DRONE = "[a-z][a-z0-9_]*"   # any robot namespace (robot_config enforces this shape)
_PAYLOAD_TOPICS = "/payload[a-z0-9_]*/vehicle/(odom|state)"   # what drones take from the payload's FC


def allowed_publishers(cfg: RobotConfig) -> list[str]:
    """Topics published on this host that other hosts may receive."""
    if cfg.zenoh.role == "ground_station":
        return ["/team/.*"]
    return [f"/{cfg.namespace}/{t}" for t in _DRONE_TOPICS]


def allowed_subscribers(cfg: RobotConfig) -> list[str]:
    """Topics other hosts publish that subscribers on this host may receive.

    A drone takes team commands from the ground station and, from other
    drones, only their mission state (the mission node waits for the whole team
    at take-off height).
    """
    if cfg.zenoh.role == "ground_station":
        return [f"/{_ANY_DRONE}/{t}" for t in _DRONE_TOPICS]
    if cfg.zenoh.role == "drone":
        return ["/team/.*", _PAYLOAD_TOPICS, f"/{_ANY_DRONE}/system/mission"]
    return ["/team/.*"]


def render(cfg: RobotConfig, domain_id: int | None = None) -> dict:
    if domain_id is None:
        domain_id = cfg.ros.domain_id
    ns = cfg.namespace
    multicast: dict = {"enabled": cfg.zenoh.multicast_scouting}
    if cfg.zenoh.multicast_scouting:
        # zenoh routers never connect to what they discover unless told to
        # (default autoconnect.router is []); without this, two drones on one
        # Wi-Fi find each other and still don't talk.
        multicast["autoconnect"] = {"router": ["router"]}
    return {
        "mode": "router",
        "connect": {"endpoints": list(cfg.zenoh.connect)},
        "listen": {"endpoints": [f"tcp/{cfg.zenoh.listen_address}:{cfg.zenoh.listen_port}"]},
        "scouting": {"multicast": multicast},
        "plugins": {
            "ros2dds": {
                "nodename": "zenoh_bridge",
                "domain": domain_id,
                # Not LOCALHOST: in that mode the bridge's Cyclone pings 127.0.0.1 but
                # advertises a non-loopback address, and never finds a Fast DDS node
                # that starts after it. Loopback-only comes from CYCLONEDDS_URI instead.
                "ros_automatic_discovery_range": "SUBNET",
                # An interface type that is listed empty is not routed at all.
                "allow": {
                    "publishers": allowed_publishers(cfg),
                    "subscribers": allowed_subscribers(cfg),
                    "service_servers": [],
                    "service_clients": [],
                    "action_servers": [],
                    "action_clients": [],
                },
                "pub_max_frequencies": [f"/{ns}/vehicle/odom={ODOM_MAX_HZ}"],
            }
        },
    }


def write(cfg: RobotConfig, path: str) -> None:
    """Write atomically so the bridge never reads a half-written file."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = f"{path}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(render(cfg), f, indent=2)
        f.write("\n")
    os.replace(tmp, path)
