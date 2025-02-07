# Based on: https://github.com/ros2/rosbag2/blob/rolling/ros2bag/ros2bag/verb/record.py
# Storage functionality: https://github.com/ros2/rosbag2/tree/rolling/rosbag2_storage_mcap
# rosbag2_py package: https://github.com/ros2/rosbag2/tree/rolling/rosbag2_py

# Base services: https://github.com/ros2/rosbag2/tree/rolling#controlling-recordings-via-services

import os
import datetime
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rosbag2_py import Recorder, StorageOptions, RecordOptions
from std_msgs.msg import *
from std_srvs.srv import Trigger, Trigger_Request, Trigger_Response

from dtcvc_scorekeeper_types.srv import (
    StateCheck,
    StateCheck_Request,
    StateCheck_Response,
)


class StoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check regularly for the stopped() condition."""

    def __init__(self, *args, **kwargs):
        super(StoppableThread, self).__init__(*args, **kwargs)
        self._stop_event = threading.Event()
        self.has_ran = False

    def stop(self):
        """Stops recording and thread"""
        Recorder.cancel()
        self._stop_event.set()

    def stopped(self):
        """Checks if thread is stopped"""
        return self._stop_event.is_set()


class DtcvcRecorder(Node):
    """Node that records the simulation's message data and saves it to a bag file for future playback"""

    def __init__(self):
        super().__init__(
            "dtcvc_recorder_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Set callback group to allow for multiple concurrent callbacks
        cb_group = ReentrantCallbackGroup()

        # Set parameters
        self.use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        self.record_on_startup = self.get_parameter("record_on_startup").get_parameter_value().bool_value
        self.recording_file_name = self.get_parameter("recording_file_name").get_parameter_value().string_value
        self.enable_scenario_recording_mode = (
            self.get_parameter("enable_scenario_recording_mode").get_parameter_value().bool_value
        )
        self.enable_sensor_recording = self.get_parameter("enable_sensor_recording").get_parameter_value().bool_value
        self.enable_simulation_topic_recording = (
            self.get_parameter("enable_simulation_topic_recording").get_parameter_value().bool_value
        )
        self.enable_competition_topic_recording = (
            self.get_parameter("enable_competition_topic_recording").get_parameter_value().bool_value
        )

        # Set file location based on mode
        if self.enable_scenario_recording_mode:
            self.recording_file_path = f"/opt/scenario-recordings/{self.recording_file_name}"
        else:
            self.recording_file_path = f"/opt/ros-recordings/{self.recording_file_name}"

        # Verify that the URI is not occupied
        if os.path.isdir(self.recording_file_path) or os.path.isfile(self.recording_file_path):
            raise Exception(f'Output directory/file "{self.recording_file_path}" already exists.')

        # Set recording thread
        self.recording_thread = StoppableThread(target=self.start_recording_thread_task)

        # Create services to start and stop recording
        self.stop_recording_service = self.create_service(
            Trigger,
            f"{self.get_name()}/stop_recording",
            self.stop_recording_callback,
            callback_group=cb_group,
        )
        self.start_recording_service = self.create_service(
            Trigger,
            f"{self.get_name()}/start_recording",
            self.start_recording_callback,
            callback_group=cb_group,
        )

        # Create service to check recording state
        self.is_recording_active_service = self.create_service(
            StateCheck,
            "/state_checks/is_recording_active",
            self.is_recording_active_callback,
            callback_group=cb_group,
        )

        # Start recorder on startup
        if self.record_on_startup:
            # Wait for the simulation start signal from the simulator

            # Handles simulation start signal
            self.comp_start_subscriber = self.create_subscription(
                Empty, "/dtc/simulation_start", self.comp_start_callback, qos_profile=10
            )
            self.original_start_time = -1.0

        # Handles simulation stop signal
        self.comp_stop_subscriber = self.create_subscription(
            Empty, "/dtc/simulation_stop", self.comp_stop_callback, qos_profile=10
        )

    def comp_start_callback(self, msg):
        self.get_logger().info("comp_start_callback(): Received [simulation_start] signal")
        self.start_recording()

    def comp_stop_callback(self, msg):
        self.get_logger().info("comp_stop_callback(): Received [simulation_stop] signal")
        # Perform any necessary cleanup before shutting down
        if self.enable_scenario_recording_mode:
            # The recorder node should begin system shutdown since the scorekeeper node is not present in a simrec deployment
            rclpy.shutdown()

    def start_recording(self):
        """Starts recorder on a new thread (allows services to communicate properly)"""
        self.recording_thread.start()
        self.recording_thread.has_ran = True

    def start_recording_callback(self, request: Trigger_Request, response: Trigger_Response):
        """Handles the '~/start_recording' service request by starting the recorder"""
        if self.recording_thread.is_alive():
            self.get_logger().info("Recorder start attempted: Recorder node is already active")
            response.success = False
            response.message = "Recorder node is already active"
        elif self.recording_thread.has_ran:
            self.get_logger().info(
                "Recorder start attempted: Recorder node cannot be ran again after being stopped, consider pausing instead."
            )
            response.success = False
            response.message = "Recorder node cannot be ran again after being stopped, consider pausing instead."
        else:
            try:
                self.get_logger().info("Recording starting...")
                self.start_recording()
                response.success = True
                response.message = "Recording started successfully"
            except Exception as e:
                response.success = False
                response.message = str(e)

        return response

    def start_recording_thread_task(self):
        """Thread task that initializes and starts the recorder"""

        # https://github.com/facebook/zstd
        # zstd (zStandard) is a lossless compression algorithm that reduces bag sizes by 25-65%
        # Define storage options
        storage_options = StorageOptions(
            uri=self.recording_file_path,
            storage_id="mcap",
            storage_preset_profile="zstd_fast",  # Can be {"none", "fastwrite", "zstd_fast", "zstd_small"}
            max_bagfile_size=0,
            max_bagfile_duration=0,
            max_cache_size=500 * 1024 * 1024,  # 500 MB
            snapshot_mode=False,  # Keeps all recordings in memory until recording ends
            custom_data={},
        )

        patterns = []

        # No sensor topics
        if not self.enable_sensor_recording:
            self.get_logger().info("Sensor recording disabled")
            visual_cameras_regex = r"/carla/.*/.*/(camera_info|image|events)"
            nonvisual_sensors_regex = r"/carla/.*/(lidar|odometry|gnss|imu|semantic_lidar|radar|radar_front)"
            patterns.append(f"({visual_cameras_regex}|{nonvisual_sensors_regex})")

        # No competition topics
        if not self.enable_competition_topic_recording:
            self.get_logger().info("Competition topic recording disabled")
            competitor_regex = r"/competitor/(.*)"
            internal_regex = r"/internal/(.*)"
            patterns.append(f"({competitor_regex}|{internal_regex})")

        # No simulation topics
        if not self.enable_simulation_topic_recording:
            self.get_logger().info("Simulation topic recording disabled")
            patterns.append(r"/carla/(.*)")

        exclude_regex = f"({'|'.join(patterns)})" if len(patterns) != 0 else ""

        # Define recording options
        record_options = RecordOptions()
        record_options.all = True
        record_options.is_discovery_disabled = False
        record_options.topics = []  # Specify topics or leave empty to record all topics
        record_options.rmw_serialization_format = ""
        record_options.topic_polling_interval = datetime.timedelta(milliseconds=100)
        record_options.regex = ""
        record_options.exclude = exclude_regex  # Regex for topics excluded from being recorded
        record_options.node_prefix = ""
        record_options.compression_mode = "NONE"  # Can be {"NONE", "FILE", "MESSAGE"}
        record_options.compression_format = ""  # Can be {"", "zstd"}
        record_options.compression_queue_size = 1
        record_options.compression_threads = 0
        record_options.topic_qos_profile_overrides = {}
        record_options.include_hidden_topics = False
        record_options.include_unpublished_topics = False
        record_options.start_paused = False
        record_options.ignore_leaf_topics = False
        record_options.use_sim_time = self.use_sim_time

        try:
            self.recorder = Recorder()
            self.recorder.record(storage_options, record_options, f"{self.get_name()}_internal")
        except KeyboardInterrupt:
            pass

    def stop_recording(self):
        """Passes a cancel request to the recorder running in the thread and kills the thread"""
        if self.recording_thread.is_alive():
            self.recording_thread.stop()
            self.recording_thread.join()
            self.get_logger().info("stop_recording(): Recording stopped")

    def stop_recording_callback(self, request: Trigger_Request, response: Trigger_Response):
        """Handles the '~/stop_recording' service request by stopping the recorder"""
        if self.recording_thread.is_alive():
            try:
                self.get_logger().info("Recording stopping...")
                self.stop_recording()
                response.success = True
                response.message = "Recording stopped successfully"
            except Exception as e:
                response.success = False
                response.message = str(e)
        else:
            self.get_logger().info("Recorder stop attempted: Recorder node not active")
            response.success = False
            response.message = "Recorder node not active"

        return response

    def is_recording_active_callback(self, request: StateCheck_Request, response: StateCheck_Response):
        """Handles the '~/is_recording_active' service request by checking if the recording thread is active"""
        try:
            if self.recording_thread.is_alive():
                response.message = "Recording is active"
                response.is_active = True
            else:
                response.message = "Recording is not active"
                response.is_active = False
            response.success = True
        except Exception as e:
            response.success = False
            response.is_active = False
            response.message = str(e)

        return response

    def __get_topic_exclude_regex(self) -> str:
        """Private function that returns the regex of what topics to exclude based on parameters"""
        patterns = []

        # No sensor topics
        if not self.enable_sensor_recording:
            self.get_logger().info("Sensor recording disabled")
            visual_cameras_regex = r"/carla/.*/.*/(camera_info|image|events)"
            nonvisual_sensors_regex = r"/carla/.*/(lidar|odometry|gnss|imu|semantic_lidar|radar)"
            patterns.append(f"({visual_cameras_regex}|{nonvisual_sensors_regex})")

        # No competition topics
        if not self.enable_competition_topic_recording:
            self.get_logger().info("Competition topic recording disabled")
            competitor_regex = r"/competitor/(.*)"
            internal_regex = r"/internal/(.*)"
            patterns.append(f"({competitor_regex}|{internal_regex})")

        # No simulation topics
        if not self.enable_simulation_topic_recording:
            self.get_logger().info("Simulation topic recording disabled")
            patterns.append(r"/carla/(.*)")

        return f"({'|'.join(patterns)})" if len(patterns) != 0 else ""


def main(args=None):
    rclpy.init(args=args)
    dtcvc_recorder_node = DtcvcRecorder()

    try:
        rclpy.spin(dtcvc_recorder_node)
    except SystemExit:
        print("main(): SystemExit caught in [recorder-node]")
    except KeyboardInterrupt:
        print("main(): KeyboardInterrupt caught in [recorder-node]")
    finally:
        print("main(): Awaiting shutdown for the [recorder-node]")
        print("main(): Stopping the recording")
        dtcvc_recorder_node.stop_recording()


if __name__ == "__main__":
    main()
