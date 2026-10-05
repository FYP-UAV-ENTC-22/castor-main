"""Planning component: the onboard flycrane policy runner.

The team slot (and so the one-hot agent id) comes from robot.yaml's
team.size / team.index. A trained model is mounted at /var/lib/castor/models
(see docker/docker-compose.prod.yml); without one the runner still starts and
reports model_loaded=false on <ns>/planning/status.

It flies only when the system layer enables it on <ns>/planning/command, and
then publishes <ns>/vehicle/setpoint. World-frame state comes from
own_state_prefix / payload_state_prefix: the simulator's ground truth for now.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from castor_common.launch_helpers import RESPAWN_DELAY_S, component_group, robot


def generate_launch_description():
    cfg = robot()
    runner = Node(
        package="castor_policy",
        executable="policy_runner",
        name="policy_runner",
        parameters=[{
            "team_size": cfg.team_size,
            "team_index": cfg.team_index,
            "robot_namespace": cfg.namespace,
            "own_state_prefix": LaunchConfiguration("own_state_prefix"),
            "payload_state_prefix": LaunchConfiguration("payload_state_prefix"),
            "model_path": LaunchConfiguration("model_path"),
            "rate_hz": LaunchConfiguration("rate_hz"),
            "sched_fifo_priority": LaunchConfiguration("sched_fifo_priority"),
            "run_inference_every_step": LaunchConfiguration("run_inference_every_step"),
        }],
        output="screen",
        respawn=True,
        respawn_delay=RESPAWN_DELAY_S,
    )
    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="/var/lib/castor/models/policy.onnx"),
        DeclareLaunchArgument("rate_hz", default_value="50.0", description="must match the policy's training rate"),
        DeclareLaunchArgument("own_state_prefix", default_value=f"/sim/{cfg.namespace}/state",
                              description="world-frame pose, twist (body rates), twist_inertial of this drone"),
        DeclareLaunchArgument("payload_state_prefix", default_value="/sim/payload/state",
                              description="world-frame payload pose"),
        DeclareLaunchArgument("sched_fifo_priority", default_value="0",
                              description="SCHED_FIFO priority for the control loop; 0 = normal scheduling"),
        DeclareLaunchArgument("run_inference_every_step", default_value="false",
                              description="run the model every step on a zero observation (timing)"),
        component_group("planning", [runner]),
    ])
