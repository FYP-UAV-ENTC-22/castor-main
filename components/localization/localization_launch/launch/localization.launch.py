"""Localization component. Empty for now: only the heartbeat runs, so the
container is already on the ROS graph and the supervisor can see it."""

from launch import LaunchDescription

from castor_common.launch_helpers import component_group


def generate_launch_description():
    return LaunchDescription([
        component_group("localization", []),
    ])
