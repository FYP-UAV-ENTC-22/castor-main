"""System component: the supervisor (YASMIN + behaviour trees), the mission node and optional recording.

  record:=true            record this robot's topics to MCAP under /var/log/castor/bags
  takeoff_height:=2.0     mission node: metres above home before handing over to RAPTOR
  use_sim_time:=false     mission node: time its states on the simulator's /clock (stack_sim)

The mission node is the temporary take-off / MARL / hold / land state machine. It
commands nothing unless the ground station sends /team/command "takeoff", and its
commands reach PX4 only if the vehicle component forwards them (enable_commands).
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from castor_common.launch_helpers import RESPAWN_DELAY_S, component_group, robot


def generate_launch_description():
    cfg = robot()
    supervisor = Node(
        package="castor_supervisor",
        executable="supervisor",
        name="supervisor",
        parameters=[{
            "robot_id": cfg.robot_id,
            "robot_namespace": cfg.namespace,
            "fc_enabled": cfg.fc.enabled,
            "update_gate_path": LaunchConfiguration("update_gate_path"),
            "mission_tree": PathJoinSubstitution([FindPackageShare("castor_supervisor"), "trees", "mission.xml"]),
        }],
        output="screen",
        respawn=True,
        respawn_delay=RESPAWN_DELAY_S,
    )

    mission = Node(
        package="castor_supervisor",
        executable="mission",
        name="mission",
        parameters=[{
            "robot_id": cfg.robot_id,
            "robot_namespace": cfg.namespace,
            "team_size": cfg.team_size,
            "team_index": cfg.team_index,
            "takeoff_height": LaunchConfiguration("takeoff_height"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        output="screen",
        respawn=True,
        respawn_delay=RESPAWN_DELAY_S,
    )

    bag_dir = os.path.join("/var/log/castor/bags", f"{cfg.namespace}_{datetime.now():%Y%m%d_%H%M%S}")
    recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("record")),
        cmd=["ros2", "bag", "record", "--storage", "mcap", "--output", bag_dir,
             "--regex", f"^/{cfg.namespace}/.*|^/team/.*"],
        name="recorder",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("update_gate_path", default_value="/run/castor/update_gate"),
        DeclareLaunchArgument("takeoff_height", default_value="2.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        component_group("system", [supervisor, mission, recorder]),
    ])
