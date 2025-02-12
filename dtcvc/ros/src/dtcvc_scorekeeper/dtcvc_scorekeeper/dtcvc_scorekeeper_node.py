"""Receives reports from the competitor, validates them, and passes them to the triage scorer."""

import json
import os
import sys
import time
from typing import Any

import pandas as pd
import rclpy
from rclpy.clock import Clock, ClockType
from jsonschema import validate, ValidationError
from rclpy.node import Node
from std_msgs.msg import Empty, String

import triage_scorer.scoring as ts
from dtcvc_scorekeeper_types.msg import *
from triage_scorer import loading
from triage_scorer.constants import enumerations as enums


class DtcvcScorekeeper(Node):
    TOPIC_SIMULATION_START = "/dtc/simulation_start"
    TOPIC_SIMULATION_STOP = "/dtc/simulation_stop"
    TOPIC_SCOREKEEPER_READY = "/dtc/scorekeeper_ready"
    TOPIC_TRIAGE_REPORT = "/competitor/drone_results"

    def __init__(self):
        super().__init__(
            "dtcvc_scorekeeper_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self._scenario_file = self.get_parameter("scenario_file").get_parameter_value().string_value
        self._ground_truth_path = self.get_parameter("ground_truth_path").get_parameter_value().string_value
        self._maps_path = self.get_parameter("maps_path").get_parameter_value().string_value
        self._logs_path = self.get_parameter("logs_path").get_parameter_value().string_value
        self._casualty_id_cap: int = self.get_parameter("casualty_id_cap").get_parameter_value().integer_value
        self._gts: list[tuple[tuple[float, float, float], pd.DataFrame]] = self.map_casualty_location_to_ground_truth()
        self._start_time: float = 0.0
        self._score_tracker = ts.ScoreTracker(self._gts, self._casualty_id_cap)
        self._reports: tuple[dict, enums.ReportRejection] = []

        self.init_topics()
        self.publish_ready_state()

    def map_casualty_location_to_ground_truth(self) -> list[tuple[tuple[float, float, float], pd.DataFrame]]:
        """Maps a GPS location (lon/lat/alt) to the ground truth data of a casualty.

        Returns:
            list: A list of mappings of casualty locations to ground truth data
        """
        casualty_loc_to_gt: list[tuple[tuple[float, float, float], pd.DataFrame]] = []
        scenario: dict | None = loading.load_scenario(self._scenario_file)
        maps: dict[str, dict[str, tuple[float, float, float]]] = loading.load_maps(self._maps_path)
        gps_zone_locations: dict[str, tuple[float, float, float]] = {}
        ground_truth: dict[str, pd.DataFrame] = loading.load_ground_truth(self._ground_truth_path)

        if scenario is not None:
            scenario_map_name: str = str(scenario["environment"]["map"])

            for map_name in maps:
                if map_name in scenario_map_name:
                    gps_zone_locations = maps[map_name]
                    break

            scenario_casualties = scenario["casualties"]

            if gps_zone_locations:
                for casualty in scenario_casualties:
                    zone: str = str(scenario_casualties[casualty]["zone"])
                    zone_loc: tuple[float, float, float] = gps_zone_locations[zone]

                    casualty_type = scenario_casualties[casualty]["casualty_type"]
                    gt = ground_truth[casualty_type]

                    casualty_loc_to_gt.append((zone_loc, gt))
        else:
            self.get_logger().error("map_casualty_location_to_ground_truth(): Failed to load scenario")
            sys.exit()

        return casualty_loc_to_gt

    def init_topics(self) -> None:
        """Initializes the publisher and subscriptions to their respective topics."""
        self.get_logger().info("init_topics(): Initializing the subscriptions and publishers")
        # Subscription receives a sim time update when the competition starts
        self.comp_start_subscription = self.create_subscription(
            Empty,
            DtcvcScorekeeper.TOPIC_SIMULATION_START,
            self.comp_start_callback,
            qos_profile=10,
        )
        # Subscription receives a report from the competitor
        self.casualty_submission_subscription = self.create_subscription(
            String,
            DtcvcScorekeeper.TOPIC_TRIAGE_REPORT,
            self.drone_results_callback,
            qos_profile=10,
        )
        # Subscription receives a stop competition signal
        self.stop_comp_subscription = self.create_subscription(
            Empty,
            DtcvcScorekeeper.TOPIC_SIMULATION_STOP,
            self.stop_comp_callback,
            qos_profile=10,
        )

        # Publisher sends scorekeeper ready signal
        self.ready_publisher = self.create_publisher(Empty, DtcvcScorekeeper.TOPIC_SCOREKEEPER_READY, 10)
        self.ready_timer = self.create_timer(1.0, self.publish_ready_state, clock=Clock(clock_type=ClockType.SYSTEM_TIME))

    def comp_start_callback(self, msg: Empty) -> None:
        """Sets the start time for the competition."""
        self.get_logger().info(f"comp_start_callback(): Received [simulation_start] signal")
        self.ready_timer.cancel()
        # Start time is based on simulation time
        self._start_time = self.get_clock().now().nanoseconds / float(10**9)
        self.get_logger().info(f"comp_start_callback(): start time | {self._start_time}")

    def drone_results_callback(self, msg: String) -> None:
        """Handles competitor casualty report submissions.

        Args:
            msg (str): A casualty report sent from the competitor node
        """
        report_rcv_time: float = self.get_clock().now().nanoseconds / float(10**9)
        self.get_logger().info(f"drone_results_callback(): report_rcv_time (ns) | {report_rcv_time}")

        report: dict = json.loads(msg.data)

        self.get_logger().info(str(report))

        competitor_report_schema: Any | None = None
        filename: str = "triage_scorer/schemas/schema.json"
        base_path: str = os.getenv("DTCVC_SCORING_LIB_PATH", "/opt/dtcvc-scorekeeper/lib/dtcvc-scoring")
        schema_path: str = os.path.join(base_path, filename)

        with open(schema_path, "r") as f:
            competitor_report_schema = json.load(f)

        try:
            # Note: The jsonschema should validate reports such that all required fields and at least three health assessment fields are present.
            validate(instance=report, schema=competitor_report_schema)
            self.get_logger().info("drone_results_callback(): Validation success")

            report_time: float = abs(self._start_time - report_rcv_time)
            self.get_logger().info(f"drone_results_callback(): report_time | {report_time}")
            err = self._score_tracker.track_report(report, report_time)
            self._reports.append((report, err))
        except ValidationError as err:
            self.get_logger().error(f"drone_results_callback(): Failed to validate message from competitor | {msg}")
            self.get_logger().error(err)
            self._reports.append((report, enums.ReportRejection.INVALID_REPORT))

    def stop_comp_callback(self, msg: Empty) -> None:
        """Handles the stopping of the competition."""
        self.get_logger().info("stop_comp_callback(): Received [simulation_stop] signal")
        runtime = loading.get_simulation_runtime(self._scenario_file)

        final_report_filepath: str = os.path.join(self._logs_path, f"final_score_report_{time.time_ns()}.json")
        self.get_logger().info(f"stop_comp_callback(): Generating final report with filename | {final_report_filepath}")
        ts.generate_final_report(final_report_filepath, self._score_tracker, self._reports, runtime)

        # Shutdown the system
        self.get_logger().info("stop_comp_callback(): Shutting down the system with rclpy.shutdown()")
        rclpy.shutdown()

    def publish_ready_state(self) -> None:
        """Publishes ready state, either True or False, on the /dtc/scorekeeper_ready topic."""
        self.get_logger().debug("publish_ready_state(): Sending [scorekeeper-node] ready signal")
        self.ready_publisher.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    dtcvc_scorekeeper_node = DtcvcScorekeeper()

    try:
        rclpy.spin(dtcvc_scorekeeper_node)
    except SystemExit:
        print("main(): SystemExit caught in [scorekeeper-node]")
    except KeyboardInterrupt:
        print("main(): KeyboardInterrupt caught in [scorekeeper-node]")
    finally:
        print("main(): [scorekeeper-node] shutting down")


if __name__ == "__main__":
    main()
