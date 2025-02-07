import time
import subprocess

from rclpy.node import Node
from std_msgs.msg import Empty

class ControllerManagerNode(Node):
    """
    This class is used to startup or shutdown vehicle controllers.
    """

    def __init__(self):
        super().__init__("controller_manager_node")
        self.timeout = 10  # Wait up to 10 seconds to shutdown a preexisting controller if necessary
        self.sleep_period = 0.05  # seconds
        self.arducopter_count = 0
        self.get_logger().info("ControllerManagerNode started!")

    def load_vehicle_controller(self, client, vehicle_name, controller_settings):
        """
        Parameters
        ----------
        client : carla.Client
            The Carla client object
        vehicle_name : str
            The name of the vehicle for which to load the controller
        controller_settings : dict
            YAML node containing controller settings (refer to scenario yaml file)
        """
        self.get_logger().info("Loading controller settings for the '%s' vehicle." % vehicle_name)

        # Check if there is a preexisting controller node running.
        previous_controller_exists = False

        namespace = self.get_namespace()
        if namespace == '/':
            namespace = ""
        
        control_node_name = namespace + "/" + vehicle_name + "_controller"
        node_namespaces = self.get_node_names_and_namespaces()

        for name, namespace in node_namespaces:
            if namespace + name == control_node_name:
                previous_controller_exists = True
                break
        
        # If there is a preexisting controller node running, shut it down.
        if previous_controller_exists:
            self.get_logger().info("Found preexisting controller '%s'. Attempting to restart with desired params..." % control_node_name)
            shutdown_topic_name = vehicle_name + "_controller/shutdown"
            shutdown_pub = self.create_publisher(Empty, shutdown_topic_name, 1)

            time_waiting = 0
            while self.count_subscribers(shutdown_topic_name) < 1:
                if time_waiting >= self.timeout:
                    self.get_logger().warn("Could not connect to the controller node's shutdown topic. Aborting...")
                    return False
                time_waiting += self.sleep_period
                time.sleep(self.sleep_period)
            
            self.get_logger().info("Successfully connected to the preexisting controller node's shutdown topic...")

            shutdown_pub.publish(Empty())

            time_waiting = 0
            while self.count_subscribers(shutdown_topic_name) > 0:
                if time_waiting >= self.timeout:
                    self.get_logger().warn("Could not shutdown the controller node. Aborting...")
                    return False
                time_waiting += self.sleep_period
                time.sleep(self.sleep_period)

            self.get_logger().info("Successfully shutdown the preexisting controller node!")

        # Check to make sure certain controller settings exist before attempting to start up the new controller node
        if controller_settings['enabled'] is False:
            self.get_logger().info("Controller for the '%s' vehicle was not enabled, so the controller node will not be launched." % vehicle_name)
            return False

        if 'package_name' not in controller_settings:
            self.get_logger().warn("Controller for the '%s' vehicle has no 'package_name' specified. Aborting...")
            return False
        
        package_name = controller_settings['package_name']

        if 'executable_name' not in controller_settings:
            self.get_logger().warn("Controller for the '%s' vehicle has no 'executable_name' specified. Aborting...")
            return False
        
        executable_name = controller_settings['executable_name']

        # Create the command that will launch the new controller node, then run it as a subprocess.
        cmd_str = 'ros2 run ' + package_name + ' ' + executable_name + \
                  ' --ros-args --remap __name:=' + vehicle_name + '_controller -p vehicle_name:=' + vehicle_name

        for param in controller_settings:
            if param != 'enabled' and param != 'package_name' and param != 'executable_name':
                cmd_str += ' -p ' + param + ':=' + str(controller_settings[param])

        if 'controller_type' in controller_settings and controller_settings['controller_type'] == 'ArdupilotQuadcopter':
            cmd_str += ' -p vehicle_number:=' + str(self.arducopter_count)
            self.arducopter_count += 1

            world_rotation_str = '0.0'
            opendrive_file = client.get_world().get_map().to_opendrive()
            index = opendrive_file.find("rotation=")
            if index != -1:
                opendrive_file_trimmed = opendrive_file[(index + 9):]
                world_rotation_str = opendrive_file_trimmed[0:opendrive_file_trimmed.find("]")]
            cmd_str += ' -p world_rotation:=' + world_rotation_str
        
        # The process should be run in the background so that it doesn't block this function from completing.
        cmd_str += ' &'

        self.get_logger().info("Starting '%s' controller using the following command:\n%s" % (vehicle_name, cmd_str))
        subprocess.run(cmd_str, executable="/bin/bash", shell=True)
        self.get_logger().info("Successfully started the '%s' controller!" % vehicle_name)

        return True