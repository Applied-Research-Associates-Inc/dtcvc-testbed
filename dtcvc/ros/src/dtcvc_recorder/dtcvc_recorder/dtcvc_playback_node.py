# Based on: https://github.com/ros2/rosbag2/blob/rolling/ros2bag/ros2bag/verb/play.py
# Storage functionality: https://github.com/ros2/rosbag2/tree/rolling/rosbag2_storage_mcap
# rosbag2_py package: https://github.com/ros2/rosbag2/tree/rolling/rosbag2_py

# Base services: https://github.com/ros2/rosbag2/tree/rolling#replaying-data

import math
import os

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.exceptions import *
from rclpy.node import Node
from rosbag2_py import Player, PlayOptions, StorageOptions, Info, BagMetadata
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from dtcvc_scorekeeper_types.srv import StateCheck


class DtcvcPlaybackNode(Node):
    """Node that plays back a recorded simulation from a bag file for review"""

    def __init__(self):
        super().__init__(
            "dtcvc_playback_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Set callback group to allow for multiple concurrent callbacks
        cb_group = ReentrantCallbackGroup()

        # Set parameters
        self.playback_on_startup = self.get_parameter("playback_on_startup").get_parameter_value().bool_value
        self.playback_file_name = self.get_parameter("playback_file_name").get_parameter_value().string_value
        self.simulation_replay_rate = self.get_parameter("simulation_replay_rate").get_parameter_value().double_value
        self.playback_queue_size = self.get_parameter("playback_queue_size").get_parameter_value().integer_value

        self.playback_file_path = f"/opt/scenario-recordings/{self.playback_file_name}"

        # Verify that the URI is not occupied
        if not os.path.isdir(self.playback_file_path):
            raise Exception(f"Bag file recording {self.playback_file_path} does not exist")

        # Ensure that queue size is large enough to run competition
        calculated_queue_size = self.__calculate_queue_size()
        if self.playback_queue_size < calculated_queue_size:
            self.get_logger().warn(
                f"Provided queue size of {self.playback_queue_size} too small, {calculated_queue_size} used instead. "
            )
            self.playback_queue_size = calculated_queue_size

        self.get_logger().info(f"Queue size has been set to: {self.playback_queue_size}")

        # Set player variables
        self.playback_running = False
        self.player = None

        # Create service to start playback and check playback state
        self.start_playback_service = self.create_service(
            Trigger,
            f"{self.get_name()}/start_playback",
            self.start_playback_callback,
            callback_group=cb_group,
        )
        self.is_playback_active_service = self.create_service(
            StateCheck,
            "/state_checks/is_playback_active",
            self.is_playback_active_callback,
            callback_group=cb_group,
        )

        # Create publisher for stop signal
        self.stop_publisher = self.create_publisher(Empty, "/dtc/simulation_stop", 10)

        # Start playback on startup
        if self.playback_on_startup:
            # Wait for the simulation start signal from the timekeeper

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
        self.start_playback()

    def comp_stop_callback(self, msg):
        self.get_logger().info("comp_stop_callback(): Received [simulation_stop] signal")
        # Perform any necessary cleanup before shutting down

    def start_playback(self):
        """Starts playback of recorded scenario"""

        self.playback_running = True

        # Define bag file options
        storage_options = StorageOptions(uri=self.playback_file_path, storage_id="mcap", storage_config_uri="")

        # Define playback options
        play_options = PlayOptions()
        play_options.read_ahead_queue_size = self.playback_queue_size
        play_options.rate = self.simulation_replay_rate
        play_options.loop = False
        play_options.topics_regex_to_exclude = ""
        play_options.clock_publish_frequency = 0.0
        play_options.delay = 0.0
        play_options.playback_duration = -1.0
        play_options.disable_keyboard_controls = True
        play_options.start_paused = False
        play_options.start_offset = 0.0
        play_options.wait_acked_timeout = -1
        play_options.disable_loan_message = False

        # Initialize player and play the bag
        if self.player == None:
            self.player = Player()

        try:
            self.player.play(storage_options, play_options)
            # Send stop signal when playback finishes
            self.stop_publisher.publish(Empty())
        except KeyboardInterrupt:
            pass
        finally:
            self.get_logger().info("Playback stopping...")
            self.playback_running = False

    def start_playback_callback(self, request: Trigger, response: Trigger):
        """Handles the '~/start_playback' service request by starting playback"""

        if self.playback_running:
            self.get_logger().info("Playback start attempted: Playback node is already active")
            response.success = False
            response.message = "Playback node is already active"
        else:
            try:
                self.get_logger().info("Playback starting...")
                self.start_playback()
                response.success = True
                response.message = "Playback started successfully"
            except Exception as e:
                response.success = False
                response.message = str(e)
        return response

    def is_playback_active_callback(self, request: StateCheck, response: StateCheck):
        """Handles the '~/is_playback_active' service request by checking if the playback thread is active"""
        try:
            if self.playback_running:
                response.message = "Playback is active"
                response.is_active = True
            else:
                response.message = "Playback is not active"
                response.is_active = False
            response.success = True
        except Exception as e:
            response.success = False
            response.is_active = False
            response.message = str(e)
        return response

    def __calculate_queue_size(self) -> int:
        """Calculates and returns the recommended size of the message queue"""

        info: BagMetadata = Info().read_metadata(self.playback_file_path, "mcap")

        simulation_duration = (info.duration.nanoseconds / 1e9) / self.simulation_replay_rate
        messages_per_second = float(info.message_count) / simulation_duration

        # Messages per second is multiplied by 15 to give ample
        # queue size for real-time playback
        return math.ceil(messages_per_second) * 15


def main(args=None):
    rclpy.init(args=args)
    node = DtcvcPlaybackNode()

    try:
        rclpy.spin(node)
    except SystemExit:
        print("main(): SystemExit caught in [playback-node]")
    except KeyboardInterrupt:
        print("main(): KeyboardInterrupt caught in [playback-node]")
    finally:
        print("main(): Awaiting shutdown for the [playback-node]")


if __name__ == "__main__":
    main()
