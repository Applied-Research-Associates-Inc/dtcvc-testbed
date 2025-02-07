import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
import cv2


class DtcvcVideoToRosNode(Node):
    def __init__(self):
        super().__init__(
            "dtcvc_video_to_ros_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.video_path = self.get_parameter_or("video_path", "/opt/scenario-video/UAV_P01_1m_F_RGB.mp4")
        if type(self.video_path) == Parameter:
            self.video_path = self.video_path.get_parameter_value().string_value

        self.role_name = self.get_parameter_or("role_name", "ego_vehicle")
        if type(self.role_name) == Parameter:
            self.role_name = self.role_name.get_parameter_value().string_value

        self.get_logger().info(f"{self.video_path} + {self.role_name}")
        self.stop_comp = False

        self.comp_stop_publisher = self.create_publisher(Empty, "/dtc/simulation_stop", 10)
        self.image_publisher = self.create_publisher(Image, "/carla/{}/rgb_front/image".format(self.role_name), 10)

        self.timer_period = 0.033333  # 33.333 milliseconds per frame or 30 FPS
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.video_path)

        self.get_logger().info("Video ready for parsing")
        self.timer = self.create_timer(self.timer_period, self.timer_callback, autostart=True)
        self.timer.reset()

    def timer_callback(self):
        if not self.stop_comp:
            ret, frame = self.cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.get_logger().info("Message publishing")
                self.image_publisher.publish(msg)
            else:
                self.get_logger().info("Video finished or failed, shutting down node.")
                self.stop_comp = True
                self.timer.cancel()
                self.comp_stop_publisher.publish("Video stream completed.")


def main(args=None):
    rclpy.init(args=args)
    dtcvc_video_to_ros_node = DtcvcVideoToRosNode()
    rclpy.spin(dtcvc_video_to_ros_node)
    dtcvc_video_to_ros_node.cap.release()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
