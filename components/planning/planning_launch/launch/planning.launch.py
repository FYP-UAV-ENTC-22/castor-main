"""Planning component: the onboard flycrane policy runner.

The team slot (and so the one-hot agent id) comes from robot.yaml's
team.size / team.index. A trained model is mounted at /var/lib/castor/models
(see docker/docker-compose.prod.yml); without one the runner still starts and
reports model_loaded=false on <ns>/planning/status.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from castor_common.launch_helpers import component_group, robot


def generate_launch_description():
    cfg = robot()
    runner = Node(
        package="castor_policy",
        executable="policy_runner",
        name="policy_runner",
        parameters=[{
            "team_size": cfg.team_size,
            "team_index": cfg.team_index,
            "model_path": LaunchConfiguration("model_path"),
            "rate_hz": LaunchConfiguration("rate_hz"),
            "sched_fifo_priority": LaunchConfiguration("sched_fifo_priority"),
        }],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="/var/lib/castor/models/policy.onnx"),
        DeclareLaunchArgument("rate_hz", default_value="100.0"),
        DeclareLaunchArgument("sched_fifo_priority", default_value="0",
                              description="SCHED_FIFO priority for the control loop; 0 = normal scheduling"),
        component_group("planning", [runner]),
    ])
