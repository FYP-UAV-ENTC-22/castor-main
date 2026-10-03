"""Vehicle component: the PX4 link and everything that talks to the flight controller.

Driven entirely by robot.yaml (see deploy/robot.example.yaml):

  fc.enabled          start the Micro XRCE-DDS agent (serial for a real FC, udp for PX4 SITL)
  mavlink.enabled     start mavlink-router: FC MAVLink port -> UDP, for QGroundControl
  fc.px4_namespace    the uxrce_dds_client namespace set on the FC ("" = /fmu/...)

The agent respawns, so the container keeps running with the FC unplugged and
picks the link up when it appears. Setpoint forwarding to PX4 stays off unless
launched with enable_setpoint_output:=true.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from castor_common.launch_helpers import RESPAWN_DELAY_S, component_group, robot


def generate_launch_description():
    cfg = robot()
    fc, mav = cfg.fc, cfg.mavlink
    enable_setpoints = LaunchConfiguration("enable_setpoint_output")

    actions = [
        Node(
            package="castor_vehicle_interface",
            executable="vehicle_interface",
            name="vehicle_interface",
            parameters=[{
                "px4_namespace": fc.px4_namespace,
                "world_frame": f"{cfg.namespace}/odom",
                "body_frame": f"{cfg.namespace}/base_link",
                "enable_setpoint_output": enable_setpoints,
            }],
            output="screen",
            respawn=True,
            respawn_delay=RESPAWN_DELAY_S,
        ),
    ]

    if fc.enabled:
        if fc.transport == "serial":
            agent_cmd = ["castor-xrce-agent", "serial", "--dev", fc.device, "-b", str(fc.baud)]
        else:
            agent_cmd = ["castor-xrce-agent", "udp4", "-p", str(fc.udp_port)]
        actions.append(ExecuteProcess(cmd=agent_cmd, name="xrce_agent", output="screen",
                                      respawn=True, respawn_delay=2.0))
    else:
        actions.append(LogInfo(msg="fc.enabled is false in robot.yaml: no XRCE agent, vehicle state will report fc_connected=false"))

    if mav.enabled:
        router_cmd = ["mavlink-routerd"]
        for ep in mav.gcs_endpoints:
            router_cmd += ["-e", ep]
        router_cmd.append(f"{mav.device}:{mav.baud}")
        actions.append(ExecuteProcess(cmd=router_cmd, name="mavlink_router", output="screen",
                                      respawn=True, respawn_delay=2.0))

    return LaunchDescription([
        DeclareLaunchArgument("enable_setpoint_output", default_value="false",
                              description="Forward <ns>/vehicle/setpoint to PX4. Off unless a flight test needs it."),
        LogInfo(condition=IfCondition(enable_setpoints),
                msg="SETPOINT OUTPUT ENABLED: <ns>/vehicle/setpoint is forwarded to PX4 trajectory_setpoint"),
        component_group("vehicle", actions),
    ])
