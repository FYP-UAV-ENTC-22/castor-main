"""Localization component: this vehicle's state in the team's world frame.

world_frame takes the vehicle component's odometry (PX4's estimate, in PX4's own local frame) and the geodetic
origin of that frame, and publishes <ns>/localization/odom and <ns>/localization/map_to_odom in the world frame:
east-north-up about the datum, one geodetic point the whole team is given.

  datum:=<latitude>,<longitude>,<altitude>    degrees, degrees, metres AMSL

Without a datum nothing is published and planning does not fly on the vehicle's own estimate. stack_sim passes the
simulated world's origin (docker/docker-compose.stack_sim.yml). The payload's estimate is not here yet.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from castor_common.launch_helpers import RESPAWN_DELAY_S, component_group, robot


def world_frame(context):
    cfg = robot()
    text = LaunchConfiguration("datum").perform(context).strip()
    params = {"robot_namespace": cfg.namespace}
    if text:
        datum = [float(x) for x in text.split(",")]
        if len(datum) != 3:
            raise ValueError(f"datum must be <latitude>,<longitude>,<altitude>, not '{text}'")
        params["datum"] = datum
    return [Node(
        package="castor_localization",
        executable="world_frame",
        name="world_frame",
        parameters=[params],
        output="screen",
        respawn=True,
        respawn_delay=RESPAWN_DELAY_S,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("datum", default_value="",
                              description="<latitude>,<longitude>,<altitude> of the world frame's origin"),
        component_group("localization", [OpaqueFunction(function=world_frame)]),
    ])
