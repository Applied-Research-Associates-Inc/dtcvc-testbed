"""Will publish a single report and then spin forever"""

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from dtcvc_competitor.dtcvc_triage_report_generator import DtcvcTriageReportGenerator


class DtcvcScoringTestCompetitor(Node):

    def __init__(self):
        super().__init__("dtcvc_scoring_test_competitor_node")
        self.generator = DtcvcTriageReportGenerator()
        self.triage_report_publisher = self.create_publisher(String, "/competitor/drone_results", 10)

        self.generate_reports(casualty_id=0, team="team-1", system="drone-1")
        self.generate_reports(casualty_id=5, team="team-2", system="drone-2")
        self.generate_reports(casualty_id=3, team="team-3", system="drone-3")
        self.generate_reports(casualty_id=4, team="team-4", system="drone-4")
        self.generate_reports(casualty_id=9, team="team-5", system="drone-5")

    def generate_reports(self, casualty_id: int, team: str, system: str):
        """Generate each of the victim reports once.

        Args:
            drone_id (int): The ID of the drone
            casualty_id (int): The ID of the casualty
        """
        time.sleep(3)
        non_random_casualty_report = self.generator.generate_non_random_casualty_report(casualty_id=casualty_id, team=team, system=system)
        self.get_logger().info(f"non_random_casualty_report={non_random_casualty_report}")
        self.triage_report_publisher.publish(String(data=json.dumps(non_random_casualty_report)))

        time.sleep(3)
        random_casualty_report = self.generator.generate_random_casualty_report(casualty_id=casualty_id, team=team, system=system)
        self.get_logger().info(str(random_casualty_report))
        self.triage_report_publisher.publish(String(data=json.dumps(random_casualty_report)))


def main(args=None):
    rclpy.init(args=args)

    dtcvc_scoring_test_competitor_node = DtcvcScoringTestCompetitor()

    rclpy.spin(dtcvc_scoring_test_competitor_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dtcvc_scoring_test_competitor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
