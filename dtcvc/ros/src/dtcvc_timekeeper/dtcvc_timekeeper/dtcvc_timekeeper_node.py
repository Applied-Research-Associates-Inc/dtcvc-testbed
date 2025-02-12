
import rclpy
import rclpy.clock
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool, Empty, Int32

from triage_scorer import loading


class Timekeeper(Node):
    TOPIC_SIMULATION_READY = "/dtc/simulation_ready"
    TOPIC_COMPETITOR_READY = "/dtc/competitor_ready"
    TOPIC_SCOREKEEPER_READY = "/dtc/scorekeeper_ready"
    TOPIC_SIMULATION_START = "/dtc/simulation_start"
    TOPIC_SIMULATION_STOP = "/dtc/simulation_stop"
    TOPIC_ROS2BRIDGE_CLIENT_COUNT = "/client_count"

    def __init__(self):
        super().__init__("dtcvc_timekeeper_node")

        # Declare parameters

        self.declare_parameter(
            "scenario_file",
            "/scenario.yaml",
            ParameterDescriptor(
                description="Scenario file to load into the simulator",
            )
        )

        # Wait for the competitor ready signal before sending competition start signal
        self.declare_parameter(
            "wait_for_competitor",
            True,
            ParameterDescriptor(
                description="Wait for competitor ready signal before starting competition (defaults to true)"
            ),
        )

        # Wait for the scorekeeper ready signal before sending competition start signal
        self.declare_parameter(
            "wait_for_scorekeeper",
            True,
            ParameterDescriptor(
                description="Wait for scorekeeper ready signal before starting competition (defaults to true)"
            ),
        )

        # Wait for the simulation ready signal before sending competition start signal
        self.declare_parameter(
            "wait_for_simulator",
            True,
            ParameterDescriptor(
                description="Wait for simulation ready signal before starting competition (defaults to true)"
            ),
        )

        # Send competition stop signal when simulator is shutdown
        self.declare_parameter(
            "stop_on_simulator_shutdown",
            True,
            ParameterDescriptor(
                description="Send competition stop signal when simulator is shutdown (defaults to true)"
            ),
        )

        # Get parameter values
        self.scenario_file: str = self.get_parameter("scenario_file").get_parameter_value().string_value
        self.wait_for_simulator = self.get_parameter("wait_for_simulator").get_parameter_value().bool_value
        self.wait_for_competitor = self.get_parameter("wait_for_competitor").get_parameter_value().bool_value
        self.wait_for_scorekeeper = self.get_parameter("wait_for_scorekeeper").get_parameter_value().bool_value
        self.stop_on_simulator_shutdown = (
            self.get_parameter("stop_on_simulator_shutdown").get_parameter_value().bool_value
        )

        self.get_logger().debug("scenario_file: %s" % self.scenario_file)
        self.get_logger().debug("wait_for_simulator: %r" % self.wait_for_simulator)
        self.get_logger().debug("wait_for_competitor: %r" % self.wait_for_competitor)
        self.get_logger().debug("wait_for_scorekeeper: %r" % self.wait_for_scorekeeper)
        self.get_logger().debug("stop_on_simulator_shutdown: %r" % self.stop_on_simulator_shutdown)

        # Create publisher for start signal
        self.start_publisher = self.create_publisher(Empty, Timekeeper.TOPIC_SIMULATION_START, 10)

        # Create publisher for stop signal
        self.stop_publisher = self.create_publisher(Empty, Timekeeper.TOPIC_SIMULATION_STOP, 10)

        self.simulation_ready_subscription = None
        self.simulation_ready = False
        if self.wait_for_simulator or self.stop_on_simulator_shutdown:
            # Subscribe to simulation ready signal
            self.simulation_ready_subscription = self.create_subscription(
                Bool,
                Timekeeper.TOPIC_SIMULATION_READY,
                self.simulation_ready_callback,
                10,
            )

        self.competitor_ready_subscription = None
        self.competitor_ready = False
        if self.wait_for_competitor:
            # Subscribe to competitor ready signal
            self.competitor_ready_subscription = self.create_subscription(
                Empty,
                Timekeeper.TOPIC_COMPETITOR_READY,
                self.competitor_ready_callback,
                10,
            )

        self.scorekeeper_ready_subscription = None
        self.scorekeeper_ready = False
        if self.wait_for_scorekeeper:
            # Subscribe to the scorekeeper ready signal
            self.scorekeeper_ready_subscription = self.create_subscription(
                Empty,
                Timekeeper.TOPIC_SCOREKEEPER_READY,
                self.scorekeeper_ready_callback,
                10,
            )

        self.competition_running = False

        self._system_clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.SYSTEM_TIME)

    def simulation_ready_callback(self, ready: Bool):
        self.get_logger().debug("simulation ready: %r" % ready.data)

        if self.simulation_ready != ready.data:
            # Transitioning ready state
            self.simulation_ready = ready.data
            if ready.data:
                # Simulator has become ready
                self.check_all_ready()
            else:
                # Simulator is no longer ready
                self.send_stop_signal()

    def competitor_ready_callback(self, ready: Empty):
        if not self.competitor_ready:
            self.get_logger().info("competitor_ready_callback(): Competitor ready")
        self.competitor_ready = True
        self.check_all_ready()

    def scorekeeper_ready_callback(self, ready: Empty):
        if not self.scorekeeper_ready:
            self.get_logger().info("scorekeeper_ready_callback(): Scorekeeper ready")
        self.scorekeeper_ready = True
        self.check_all_ready()

    def check_all_ready(self):
        if (
            (not self.wait_for_competitor or self.competitor_ready)
            and (not self.wait_for_scorekeeper or self.scorekeeper_ready)
            and (not self.wait_for_simulator or self.simulation_ready)
        ):
            # Unsubscribe from ready signals
            if self.competitor_ready_subscription is not None:
                self.destroy_subscription(self.competitor_ready_subscription)
            if self.scorekeeper_ready_subscription is not None:
                self.destroy_subscription(self.scorekeeper_ready_subscription)
        if (
            not self.competition_running
            and (not self.wait_for_competitor or self.competitor_ready)
            and (not self.wait_for_scorekeeper or self.scorekeeper_ready)
            and (not self.wait_for_simulator or self.simulation_ready)
        ):
            self.get_logger().info("check_all_ready(): All nodes ready")
            self.competition_running = True
            self.send_start_signal()

    def send_start_signal(self):
        self.get_logger().info("send_start_signal(): Sending [simulation_start] signal")
        self.start_publisher.publish(Empty())
        self.start_competition_timer()

    def send_stop_signal(self):
        self.get_logger().info("send_stop_signal(): Sending [simulation_stop] signal")
        self.stop_publisher.publish(Empty())

    def start_competition_timer(self):
        runtime = loading.get_simulation_runtime(self.scenario_file)
        if runtime is not None:
            self.simulation_duration = runtime
            self._sim_start = self.get_clock().now()
            self._real_start = self._system_clock.now()
            # Create a timer where the duration is equal to the runtime; this timer is based off simulation time since the 'use_sim_time' parameter is set to True
            self.simulation_timer = self.create_timer(self.simulation_duration, self.simulation_timeout_callback)
            self.get_logger().info("start_competition_timer(): Starting simulation timer for %ds, current sim %dns, current real %dns" % (self.simulation_duration, self._sim_start.nanoseconds, self._real_start.nanoseconds))

    def simulation_timeout_callback(self):
        sim_end = self.get_clock().now()
        real_end = self._system_clock.now()
        sim_duration = sim_end.nanoseconds - self._sim_start.nanoseconds
        real_duration = real_end.nanoseconds - self._real_start.nanoseconds

        self.get_logger().info(f"simulation_timeout_callback(): Simulation timeout, current sim {sim_end.nanoseconds}ns, current real {real_end.nanoseconds}ns, duration sim {sim_duration / float(10**9)}s, duration real {real_duration / float(10**9)}s")
        self.simulation_timer.cancel()
        self.send_stop_signal()


def main(args=None):
    rclpy.init(args=args)
    node = Timekeeper()

    try:
        rclpy.spin(node)
    except SystemExit:
        print("main(): SystemExit caught in the [timekeeper-node]")
    except KeyboardInterrupt:
        print("main(): KeyboardInterrupt caught in the [timekeeper-node]")
    finally:
        print("main(): Awaiting shutdown for the [timekeeper-node]")


if __name__ == "__main__":
    main()
