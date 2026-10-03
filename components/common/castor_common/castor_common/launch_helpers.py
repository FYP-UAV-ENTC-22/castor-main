"""Shared pieces for the <component>_launch packages.

Every component puts its nodes under /<robot namespace>/<component>/, so a node
named `vehicle_interface` in the vehicle container on drone1 is
/drone1/vehicle/vehicle_interface and its relative topic `odom` is
/drone1/vehicle/odom.
"""

from __future__ import annotations

from functools import lru_cache

from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

from .robot_config import RobotConfig, load

# Respawn delay for ROS nodes. Longer than the Fast DDS lease (5 s,
# docker/fastdds_localhost.xml): a crashed node never leaves DDS, and if its
# replacement (same name) appears before the dead participant expires, the zenoh
# bridge retires that topic's route when the expiry comes. Plain processes that
# are not ROS nodes (the XRCE agent, mavlink-router) can respawn sooner.
RESPAWN_DELAY_S = 6.0


@lru_cache(maxsize=1)
def robot() -> RobotConfig:
    return load()


def heartbeat(component: str) -> Node:
    return Node(
        package="castor_common",
        executable="heartbeat",
        name="heartbeat",
        parameters=[{"component": component, "robot_id": robot().robot_id}],
        output="screen",
    )


def component_group(component: str, actions: list) -> GroupAction:
    """Namespace `actions` under /<robot>/<component>/ and add the heartbeat."""
    return GroupAction([
        PushRosNamespace(robot().namespace),
        PushRosNamespace(component),
        heartbeat(component),
        *actions,
    ])
