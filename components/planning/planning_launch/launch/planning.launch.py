"""Planning component: the onboard flycrane policy runner.

The team slot (and so the one-hot agent id) comes from robot.yaml's
team.size / team.index. The policy is a model package (models/README.md):
`model:=<name>/<version>`, else the DEFAULT of /var/lib/castor/models (mounted,
see docker/docker-compose.prod.yml), else the DEFAULT baked into the image at
/opt/castor/models. Its manifest sets the runner's slot file, rate, observation
width and flight settings. `model_path:=<file>.onnx` flies a bare model with the
runner's built-in defaults instead. Without any model the runner still starts
and reports model_loaded=false on <ns>/planning/status.

It flies only when the system layer enables it on <ns>/planning/command, and
then publishes <ns>/vehicle/setpoint. World-frame state comes from
own_state_prefix / payload_state_prefix: the simulator's ground truth for now.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from castor_common import models
from castor_common.launch_helpers import RESPAWN_DELAY_S, component_group, robot

LEGACY_MODEL = os.path.join(models.MOUNTED, "policy.onnx")


def runner(context):
    cfg = robot()
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    params = {
        "team_size": cfg.team_size,
        "team_index": cfg.team_index,
        "robot_namespace": cfg.namespace,
        "own_state_prefix": arg("own_state_prefix"),
        "payload_state_prefix": arg("payload_state_prefix"),
        "sched_fifo_priority": int(arg("sched_fifo_priority")),
        "run_inference_every_step": arg("run_inference_every_step").lower() == "true",
        "step_trigger": arg("step_trigger"),
        "use_sim_time": arg("use_sim_time").lower() == "true",
    }
    actions = []
    bare = arg("model_path")
    if not bare and not arg("model") and os.path.isfile(LEGACY_MODEL) and not models.default_id((models.MOUNTED,)):
        bare = LEGACY_MODEL  # mounted the old way, without a manifest: it still wins over the baked default
    if bare:
        params["model_path"] = bare
        actions.append(LogInfo(msg=f"planning: bare model {bare}, runner defaults"))
    else:
        try:
            model = models.find(arg("model") or None)
            params.update(model.runner_parameters(cfg.team_size, cfg.team_index))
            actions.append(LogInfo(msg=f"planning: model {model.id} from {model.path}, slot {cfg.team_index}"))
        except models.ModelError as e:
            params["model_path"] = ""
            actions.append(LogInfo(msg=f"planning: NO MODEL: {e}"))
    if arg("rate_hz"):
        params["rate_hz"] = float(arg("rate_hz"))
    actions.append(Node(
        package="castor_policy",
        executable="policy_runner",
        name="policy_runner",
        parameters=[params],
        output="screen",
        respawn=True,
        respawn_delay=RESPAWN_DELAY_S,
    ))
    return actions


def generate_launch_description():
    cfg = robot()
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="", description="<name>/<version> or a package directory; "
                              "empty = DEFAULT of the mounted, then the baked models"),
        DeclareLaunchArgument("model_path", default_value="", description="a bare .onnx instead of a package"),
        DeclareLaunchArgument("rate_hz", default_value="", description="override the model's training rate"),
        DeclareLaunchArgument("own_state_prefix", default_value=f"/sim/{cfg.namespace}/state",
                              description="world-frame pose, twist (body rates), twist_inertial of this drone"),
        DeclareLaunchArgument("payload_state_prefix", default_value="/sim/payload/state",
                              description="world-frame payload pose"),
        DeclareLaunchArgument("sched_fifo_priority", default_value="0",
                              description="SCHED_FIFO priority for the control loop; 0 = normal scheduling"),
        DeclareLaunchArgument("run_inference_every_step", default_value="false",
                              description="run the model every step on a zero observation (timing)"),
        DeclareLaunchArgument("step_trigger", default_value="timer",
                              description="timer (wall clock, a Pi) or payload (one step per payload sample, stack_sim)"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="the simulator's /clock (stack_sim)"),
        component_group("planning", [OpaqueFunction(function=runner)]),
    ])
