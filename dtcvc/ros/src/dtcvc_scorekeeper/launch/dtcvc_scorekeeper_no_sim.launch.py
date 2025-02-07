import datetime

import launch
import launch_ros.actions

from launch.actions import LogInfo, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
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
                "port": 8765,
                "address": "0.0.0.0",
                "tls": False,
                "certfile": "",
                "keyfile": "",
                "topic_whitelist": [".*"],
                "param_whitelist": [".*"],
                "service_whitelist": [".*"],
                "client_topic_whitelist": [".*"],
                "min_qos_depth": 1,
                "max_qos_depth": 10,
                "num_threads": LaunchConfiguration("foxglove_bridge_thread_count"),
                "send_buffer_limit": LaunchConfiguration("foxglove_bridge_buffer_limit"),
                "use_sim_time": True,
                "use_compression": True,
                "capabilities": [
                    "clientPublish",
                    "parameters",
                    "parametersSubscribe",
                    "services",
                    "connectionGraph",
                    "assets",
                ],
                "include_hidden": True,
                "asset_uri_allowlist": [
                    "^package://(?:\\w+/)*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$"
                ],
            }
        ],
        arguments=["--ros-args", "--log-level", "FATAL"],
        condition=IfCondition(LaunchConfiguration("enable_foxglove_bridge")),
    )

    dtcvc_scorekeeper_node = launch_ros.actions.Node(
        package="dtcvc_scorekeeper",
        executable="dtcvc_scorekeeper_node",
        name="dtcvc_scorekeeper_node",
        output="screen",
        emulate_tty="True",
        on_exit=launch.actions.Shutdown(),
        parameters=[
            {"use_sim_time": True},
            {"scenario_file": LaunchConfiguration("scenario_file")},
            {"enable_ground_truth_intake": LaunchConfiguration("enable_ground_truth_intake")},
            {"ground_truth_path": LaunchConfiguration("ground_truth_path")},
            {"casualty_id_cap": LaunchConfiguration("casualty_id_cap")},
        ],
    )

    dtcvc_timekeeper_node = launch_ros.actions.Node(
        package="dtcvc_timekeeper",
        executable="dtcvc_timekeeper_node",
        name="dtcvc_timekeeper_node",
        output="screen",
        emulate_tty="True",
        on_exit=launch.actions.Shutdown(),
        parameters=[
            {"use_sim_time": True},
            {"scenario_file": LaunchConfiguration("scenario_file")},
            {"wait_for_competitor": True},
            {"wait_for_scorekeeper": True},
            {"wait_for_simulator": False},
            {"stop_on_simulator_shutdown": False},
        ],
    )

    scenario_playback = launch_ros.actions.Node(
        package="dtcvc_recorder",
        executable="dtcvc_playback_node",
        name="dtcvc_playback_node",
        output="screen",
        emulate_tty="True",
        on_exit=launch.actions.Shutdown(),
        parameters=[
            {"playback_on_startup": LaunchConfiguration("playback_on_startup")},
            {"playback_file_name": LaunchConfiguration("playback_file_name")},
            {"simulation_replay_rate": LaunchConfiguration("simulation_replay_rate")},
            {"playback_queue_size": LaunchConfiguration("playback_queue_size")},
        ],
    )

    dtcvc_recorder_node = launch_ros.actions.Node(
        package="dtcvc_recorder",
        executable="dtcvc_recorder_node",
        name="dtcvc_recorder_node",
        output="screen",
        emulate_tty="True",
        on_exit=launch.actions.Shutdown(),
        parameters=[
            {"use_sim_time": True},
            {"record_on_startup": LaunchConfiguration("record_on_startup")},
            {"enable_scenario_recording_mode": False},
            {"enable_sensor_recording": LaunchConfiguration("enable_sensor_recording")},
            {"enable_simulation_topic_recording": LaunchConfiguration("enable_simulation_topic_recording")},
            {"enable_competition_topic_recording": LaunchConfiguration("enable_competition_topic_recording")},
            {"recording_file_name": LaunchConfiguration("recording_file_name")},
        ],
    )

    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="scenario_file",
                description="Scenario file to load into the simulator",
            ),
            launch.actions.DeclareLaunchArgument(
                name="enable_ground_truth_intake",
                default_value="True",
                description="Enable/Disable ground truth data extraction from 'ground_truth_path'. ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="ground_truth_path",
                default_value="/opt/ground_truth",
                description="Path to ground truth data directory.",
            ),
            launch.actions.DeclareLaunchArgument(
                name="simulation_replay_rate",
                default_value="1.0",
                description="Rate (float) at which the simulation is replayed, 1.0 for 1x speed, 0.5 for half speed, 2.0 for double speed. ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="playback_queue_size",
                default_value="10000",
                description="Amount of messages (integer) that will be prepared ahead of playback in a queue. ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="playback_on_startup",
                default_value="True",
                description="Enable/disable playing back the simulation on startup. ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="playback_file_name",
                default_value=datetime.datetime.now().strftime("scenario_recording_%Y_%m_%d-%H_%M_%S.bag"),
                description="Name of the .bag file simulation recording to playback. ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="record_on_startup",
                default_value="False",
                description="Enable/disable recording the simulation on startup. ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="enable_sensor_recording",
                default_value="True",
                description="Enable/disable recording the sensor feed data (e.g. /carla/*/rgb_camera/*). ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="enable_simulation_topic_recording",
                default_value="True",
                description="Enable/disable recording simulation topics (e.g. /carla/*). ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="enable_competition_topic_recording",
                default_value="True",
                description="Enable/disable recording competition topics (e.g. /competitor/*). ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="recording_file_name",
                default_value=datetime.datetime.now().strftime("recording_%Y_%m_%d-%H_%M_%S.bag"),
                description="Path of the file generated by the recorder. ",
            ),
            launch.actions.DeclareLaunchArgument(
                name="enable_foxglove_bridge",
                default_value="False",
                description="Enable/disable a websocket-based bridge for viewing live data with foxglove 'live connection'. ",
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
                name="casualty_id_cap",
                default_value="30",
                description="The default capacity of unique casualty IDs that would exist for a scenario.",
            ),
            scenario_playback,
            dtcvc_timekeeper_node,
            dtcvc_scorekeeper_node,
            dtcvc_recorder_node,
            foxglove_ros_bridge,
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
