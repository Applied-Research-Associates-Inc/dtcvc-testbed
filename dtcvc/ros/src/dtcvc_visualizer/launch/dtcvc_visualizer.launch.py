import datetime

import launch
import launch_ros.actions

from launch.actions import LogInfo, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart


def generate_launch_description():

    foxglove_ros_bridge = launch_ros.actions.Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="screen",
        emulate_tty="True",
        on_exit=launch.actions.Shutdown(),
        parameters=[
            {
                "port": LaunchConfiguration("foxglove_bridge_websocket_port"),
                "address": LaunchConfiguration("foxglove_bridge_connection_hostname"),
                "tls": False,
                "certfile": "",
                "keyfile": "",
                "topic_whitelist": [".*"],
                "param_whitelist": [".*"],
                "service_whitelist": [".*"],
                "client_topic_whitelist": [".*"],
                "min_qos_depth": 1,
                "max_qos_depth": 25,
                "num_threads": LaunchConfiguration("foxglove_bridge_thread_count"),
                "send_buffer_limit": LaunchConfiguration("foxglove_bridge_buffer_limit"),
                "use_sim_time": True,
                "capabilities": [
                    "clientPublish",
                    "parameters",
                    "parametersSubscribe",
                    "services",
                    "connectionGraph",
                    "assets",
                ],
                "include_hidden": True,
                "use_compression": True,
                "asset_uri_allowlist": [
                    "^package://(?:\\w+/)*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$"
                ],
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        condition=IfCondition(LaunchConfiguration("enable_foxglove_bridge")),
    )
    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="enable_foxglove_bridge",
                default_value="True",
                description="Enable/disable a websocket-based bridge for viewing live data with foxglove 'live connection'. ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="foxglove_bridge_connection_hostname",
                default_value="0.0.0.0",
                description="Hostname/IP of the server hosting the ROS2 nodes ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="foxglove_bridge_websocket_port",
                default_value="8765",
                description="Port to host websocket on. ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="foxglove_bridge_thread_count",
                default_value="0",
                description="If foxglove bridge enabled: Number of threads to be allocated to foxglove bridge (defaults to 0 which is one per core) ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="foxglove_bridge_buffer_limit",
                default_value="4000000000",
                description="If foxglove bridge enabled: Size of stream buffer allocated to foxglove bridge in bytes (defaults to 4000000000 which is 4 Gigabytes) ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="foxglove_bridge_compression_enable",
                default_value="True",
                description="Enable per-message websocket compression. ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="log_level",
                default_value="FATAL",
                description="Log level of the foxglove bridge ",
            ),
            foxglove_ros_bridge,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=foxglove_ros_bridge,
                    on_start=[LogInfo(msg="CARLA ROS Bridge started. ")],
                )
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
