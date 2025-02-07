#!/usr/bin/env python

import os
import argparse
import yaml
import logging
import carla
import rclpy
import threading
import numpy as np
import traceback
from time import sleep

from dtc_manager.TickManager import SyncState, TickManagerNode
from dtc_manager.ControllerManager import ControllerManagerNode
from dtc_manager.SensorLoadouts import SensorLoadouts as sl

from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

python_file_path = os.path.realpath(__file__)
python_file_path = python_file_path.replace("run_system_manager.py", "")
global_transform = carla.Transform(carla.Location(x=0, y=0, z=0), carla.Rotation(yaw=180))

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Online Available rpy to quat conversion
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]

class SimulationScenarioNode(Node):

    def __init__(self):
        super().__init__(
            "dtcvc_scenario_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

class SimulationStatusNode(Node):

    def __init__(self):
        super().__init__("carla_simulation_status")
        self._status = False
        self.publisher = self.create_publisher(Bool, "/dtc/simulation_ready", 10)
        self._timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        msg = Bool()
        msg.data = self._status
        self.publisher.publish(msg)

    def set_status(self, status):
        self._status = status

class SimulationStartNode(Node):

    def __init__(self):
        super().__init__(
            "carla_simulation_start", 
            parameter_overrides=[
                # 'use_sim_time' must be declared here since the node is not defined in a launch file
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        )
        self._start = False
        self._start_time = None
        self._timeout = 1800000000000  # 30 minutes in nanoseconds
        self.subscriber = self.create_subscription(Empty, "/dtc/simulation_start", self.set_start, 10)

    def get_start(self):
        """Get the start status of the node."""
        return self._start

    def check_timeout(self):
        """Checks the difference of the current simulation time and the start time against the timeout each iteration of the while-loop in main()."""
        if self._start_time is None:
            return False

        if self.get_clock().now().nanoseconds - self._start_time >= self._timeout:
            return True
        return False

    def set_start(self, msg):
        """Sets the start status to True and sets the start time."""
        self._start = True
        if self._start_time is None:
            self._start_time = self.get_clock().now().nanoseconds
            logging.info("  Start Time: %s", self._start_time)

class SimulationVehicleOdometryNode(Node):

    def __init__(self, vehicle_name):
        super().__init__(vehicle_name + "_odometry_node", 
            parameter_overrides=[
                # 'use_sim_time' must be declared here since the node is not defined in a launch file
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ])
        self._vehicle = None
        self._vehicle_name = vehicle_name
        self._simulation_started = False
        self.publisher = self.create_publisher(Odometry, "/carla/" + vehicle_name + "/odometry", 10)
        self._timer = self.create_timer(0.01666, self.publish_odom)

    def publish_odom(self):
        if self._vehicle is None or self._simulation_started == False:
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._vehicle_name
        msg.child_frame_id = self._vehicle_name

        transform = self._vehicle.get_transform()

        # Pose Y needs to be flipped
        msg.pose.pose.position.x = transform.location.x
        msg.pose.pose.position.y = -transform.location.y
        msg.pose.pose.position.z = transform.location.z

        # RPY comes in degrees, convert to rads, pitch and yaw needs to be converted to right hand rule
        roll = transform.rotation.roll * 0.0174533
        pitch = -transform.rotation.pitch * 0.0174533
        yaw = -transform.rotation.yaw * 0.0174533
        quat = get_quaternion_from_euler(roll, pitch, yaw)
        # returned [qx, qy, qz, qw]
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        # NOTE TO ANYONE READING, Twist Used because the current system doesn't use physics

        self.publisher.publish(msg)

    def set_vehicle(self, vehicle):
        self._vehicle = vehicle

    def set_start(self):
        self._simulation_started = True

def _setup_casualty_actors(world, scenario_file, bp_library):
    if "casualties" not in scenario_file:
        logging.info("  No Casualties defined in Scenario File")
        raise Exception("No Casualties defined in Scenario File")
    logging.info("  Setting Casualties...")
    functor_sent_casualties_bp = bp_library.filter("functorsendcasualties")[0]
    iteration = 1
    logging.info("  Begin iterating through Casualties...")
    for casualty_name in scenario_file["casualties"]:
        casualty = scenario_file["casualties"][casualty_name]
        if "zone" not in casualty:
            logging.info("  No Zone defined in Casualty Scenario Loadout")
            raise Exception("No Zone defined in Casualty Scenario Loadout")
        if "casualty_type" not in casualty:
            logging.info("  No Type defined in Casualty Scenario Loadout")
            raise Exception("No Type defined in Casualty Scenario Loadout")
        casualty_string = casualty["casualty_type"] + "|" + str(casualty["zone"])
        functor_sent_casualties_bp.set_attribute(str(iteration), casualty_string)
        iteration += 1
    return world.spawn_actor(functor_sent_casualties_bp, global_transform)

def get_vehicle_names(scenario_file):
    vehicle_names = []
    if 'vehicles' in scenario_file:
        for vehicle_name in scenario_file['vehicles']:
            vehicle_names.append(vehicle_name)
    return vehicle_names

def load_client_settings(client, client_settings, tmn):
    if 'timeout' in client_settings:
        client.set_timeout(client_settings['timeout'])
    else:
        client.set_timeout(60.0)

    timestep = 0.05
    if 'sim_step' in client_settings:
        timestep = client_settings['sim_step']

    sync_state = SyncState.Synchronous
    if 'synchronous_state' in client_settings:
        state = client_settings['synchronous_state']
        if state == 'synchronous':
            sync_state = SyncState.Synchronous
        elif state == 'synchronous_real_time':
            sync_state = SyncState.SynchronousRealTime
        elif state == 'synchronous_external_trigger':
            sync_state = SyncState.SynchronousExternalTrigger
        else:
            logging.warning("  Unknown synchronous state '%s' specified. Defaulting to the 'synchronous' state." % state)

    tmn.set_carla_client(client)
    tmn.set_sync_state(sync_state, timestep)

    # Carla documentation says to set the traffic manager synchronous mode to match
    # the world synchronous mode. If DTC isn't using the traffic manager though, this step
    # is probably not necessary.
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)

def load_environment_settings(client, env_settings, world_settings, tmn):
    # Change CARLA Map to desired map
    if 'map' in env_settings:
        logging.debug('  Checking for Map: %s', env_settings['map'])
        logging.debug('  Available Map: %s', client.get_available_maps())
        for map in client.get_available_maps():
            if env_settings['map'] in map:
                logging.debug('  Loading Map: %s', map)
                client.load_world(map)
                world = client.get_world()
                world.apply_settings(world_settings)

                # Get and modify the weather
                # weather = old_world.get_weather()
                # weather.cloudiness=10.000000
                # weather.precipitation=0.00000
                # weather.precipitation_deposits=0.00000
                # weather.wind_intensity=5.000000
                # weather.sun_azimuth_angle=-1.000000
                # weather.sun_altitude_angle=45.000000
                # weather.fog_density=0.000000
                # weather.fog_distance=0.750000
                # weather.fog_falloff=0.250000
                # weather.wetness=0.000000
                # weather.scattering_intensity=1.000000
                # weather.mie_scattering_scale=0.003996
                # weather.rayleigh_scattering_scale=0.033100
                # weather.dust_storm=0.000000
                # world.set_weather(weather)

                # Set the carla client within the tick_manager_node again so that its internal
                # carla_world object is updated with this new world.
                tmn.set_carla_client(client)
    else:
        logging.info("  No Map in Scenario File")
        raise Exception("Load Environment Settings failed.")

def load_vehicle_settings(client, vehicles, cmn):
    vehicle_actors = []
    sensor_actors = []
    vehicle_control_type_map = {"quadcopter":"multirotor"} # Once supported, add "diffdrive" and "ackermann"
    try:
        world = client.get_world()
        bp_library = world.get_blueprint_library()
        spawn_point_index = 0
        spawn_points = world.get_map().get_spawn_points()
        for vehicle_name in vehicles:
            logging.info("  Loading Vehicle Settings for '%s'..." % vehicle_name)
            vehicle_settings = vehicles[vehicle_name]
            if 'type' not in vehicle_settings:
                logging.info("  No 'type' specified for the '%s' vehicle. Aborting..." % vehicle_name)
                raise Exception("Load Vehicle Settings failed.")
            vehicle_control_type = vehicle_settings["type"]
            if vehicle_control_type not in vehicle_control_type_map:
                raise Exception("Unsupported vehicle control type '%s'." % vehicle_control_type)
            vehicle_type = vehicle_control_type_map[vehicle_control_type]
            logging.debug("  Spawning vehicle: %s", vehicle_type)
            vehicle_bp_library = bp_library.filter(vehicle_type)
            vehicle_bp = None
            if len(vehicle_bp_library) > 0:
                vehicle_bp = vehicle_bp_library[0]
            else:
                raise Exception("Could not find blueprint of the specified vehicle type '%s'. Was it spelled correctly?" % vehicle_type)
            # By default, Carla supports only one user-controlled vehicle that must have its role_name set to 'hero'.
            # DTC's version of Carla though allows for multiple user-controlled 'multirotor' vehicles that do not
            # need to have their 'role_name's set to 'hero'
            if vehicle_type != 'multirotor':
                vehicle_bp.set_attribute("role_name", 'hero')
            else:
                vehicle_bp.set_attribute("role_name", vehicle_name)
            vehicle_bp.set_attribute("ros_name",  vehicle_name)
            logging.debug("  Spawning vehicle in world: '%s'", vehicle_type)
            if spawn_point_index >= len(spawn_points):
                raise Exception("Ran out of spawning locations. Aborting...")
            vehicle_actor = world.spawn_actor(vehicle_bp, spawn_points[spawn_point_index], attach_to=None)
            vehicle_actors.append(vehicle_actor)
            if 'controller' in vehicles[vehicle_name]:
                cmn.load_vehicle_controller(client, vehicle_name, vehicles[vehicle_name]['controller'])
            if 'sensors' in vehicles[vehicle_name]:
                load_sensor_settings(client, vehicles[vehicle_name]['sensors'], sensor_actors, vehicle_actor)
            spawn_point_index += 1

    except Exception as error:
        logging.info('  Error: %s', type(error))
        logging.info('  Error: %s', error)
        raise Exception("Load Vehicle Settings failed.")
    
    return vehicle_actors, sensor_actors

def load_sensor_settings(client, sensors, actors, attach_to=None):
    try:
        world = client.get_world()
        bp_library = world.get_blueprint_library()
        for sensor_name in sensors:
            logging.info("  Loading Sensor Settings for '%s'." % sensor_name)
            sensor_settings = sensors[sensor_name]
            if 'type' not in sensor_settings:
                logging.info("  No 'type' specified for the '%s' sensor. Aborting..." % sensor_name)
                raise Exception("Load Sensor Settings failed.")
            manufacturer = sensor_settings['type']
            logging.debug("  Spawning sensor: %s.", manufacturer)
            if manufacturer not in sl.sensor_configs:
                raise Exception("Unsupported sensor type '%s'." % manufacturer)
            sensor_configs = sl.sensor_configs[manufacturer]
            sensor_type = sensor_configs["sensor_type"]
            sensor_bp = bp_library.filter(sensor_type)[0]
            sensor_bp.set_attribute('ros_name', sensor_name)
            sensor_bp.set_attribute('role_name', sensor_name)
            for attribute, value in sensor_configs["attributes"].items():
                sensor_bp.set_attribute(attribute, value)
            sensor_transform = carla.Transform()
            if 'x' in sensor_settings:
                sensor_transform.location.x = sensor_settings['x']
            if 'y' in sensor_settings:
                sensor_transform.location.y = sensor_settings['y']
            if 'z' in sensor_settings:
                sensor_transform.location.z = sensor_settings['z']
            if 'pitch_deg' in sensor_settings:
                sensor_transform.rotation.pitch = sensor_settings['pitch_deg']
            if 'yaw_deg' in sensor_settings:
                sensor_transform.rotation.yaw = sensor_settings['yaw_deg']
            if 'roll_deg' in sensor_settings:
                sensor_transform.rotation.roll = sensor_settings['roll_deg']
            if attach_to is None:
                logging.debug("  Spawning sensor in world: %s.", sensor_type)
            else:
                logging.debug("  Spawning sensor on vehicle: %s.", sensor_type)
            sensor_actor = world.spawn_actor(sensor_bp, sensor_transform, attach_to)
            sensor_actor.enable_for_ros()
            actors.append(sensor_actor)

    except Exception as error:
        logging.info('  Error: %s', type(error))
        logging.info('  Error: %s', error)
        raise Exception("Load Sensor Settings failed.")

def main(args=None):
    rclpy.init(args=args)
    
    # Launch Scenario Node and get params
    simulation_scenario_node = SimulationScenarioNode()

    file = simulation_scenario_node.get_parameter("file").get_parameter_value().string_value
    host = simulation_scenario_node.get_parameter("host").get_parameter_value().string_value
    port = simulation_scenario_node.get_parameter("port").get_parameter_value().integer_value
    host = host if host else "localhost"
    port = port if port else 2000
    clean_kill = simulation_scenario_node.get_parameter("clean_kill").get_parameter_value().bool_value
    verbose = simulation_scenario_node.get_parameter("verbose").get_parameter_value().bool_value

    log_level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(level=log_level)

    # Load Scenario File
    scenario_path = file if file else python_file_path + "scenarios/example.yaml"
    logging.debug(" Loading Scenario File: %s", scenario_path)
    scenario_file = yaml.safe_load(open(scenario_path, "r"))
    logging.debug("  %s", scenario_file)
    vehicle_names = get_vehicle_names(scenario_file)
    
    simulation_status_node = SimulationStatusNode()
    simulation_start_node = SimulationStartNode()
    tick_manager_node = TickManagerNode()
    controller_manager_node = ControllerManagerNode()
    odom_nodes = []
    for name in vehicle_names:
        odom_nodes.append(SimulationVehicleOdometryNode(name))
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(simulation_status_node)
    executor.add_node(simulation_start_node)
    executor.add_node(controller_manager_node)
    for node in odom_nodes:
        executor.add_node(node)
    executor.add_node(tick_manager_node)

    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    world = None
    old_world = None
    original_settings = None
    tracked_actors = []
    mission_started = False

    try:
        # Setup CARLA World
        logging.debug(" Setting up Carla Client and Settings")
        client = None
        old_world = None
        original_settings = None
        while client == None:
            try:
                logging.info(" Connecting to Carla at '%s:%d'", host, port)
                client = carla.Client(host, port)
                old_world = client.get_world()
                original_settings = old_world.get_settings()
            except RuntimeError as err:
                logging.info("  Error connecting to Carla: %s", err)
                pass
            if client == None:
                sleep(5)

        # Setup Carla Client
        load_client_settings(client, scenario_file['client'], tick_manager_node)

        # Setup Environment
        load_environment_settings(client, scenario_file['environment'], client.get_world().get_settings(), tick_manager_node)

        world = client.get_world()
        bp_library = world.get_blueprint_library()

        # Setup MetaHumans
        tracked_actors.append(_setup_casualty_actors(world, scenario_file, bp_library))

        # Create Vehicle with sensors
        vehicle_actors, sensor_actors = load_vehicle_settings(client, scenario_file['vehicles'], controller_manager_node)

        if len(vehicle_actors) != len(vehicle_names):
            raise Exception("Number of spawned vehicle actors [%d] does not match number of vehicles in the scenario file [%d]", len(vehicle_actors, len(vehicle_names)))

        for (node, actor) in zip(odom_nodes, vehicle_actors):
            node.set_vehicle(actor)
            tracked_actors.append(actor)

        for actor in sensor_actors:
           tracked_actors.append(actor)


        # Start Simulation, need to process a second of frames to fully load things
        logging.info("  Starting Simulation...")
        for step in range(int(1/tick_manager_node.get_sim_time_step())):
            _ = world.tick()
        simulation_status_node.set_status(True)

        while True:
            try:
                # Check if the simulation has not been started but should, if so, trigger the start command in Unreal
                if not mission_started and simulation_start_node.get_start():
                    logging.info("  Running Mission...")
                    mission_started = True
                    for node in odom_nodes:
                        node.set_start()

                # Tick the simulation if the mission has been started, otherwise, wait for the start command
                if mission_started and not tick_manager_node.is_tick_paused():
                    sync_state = tick_manager_node.get_sync_state()
                    if sync_state is SyncState.Synchronous or sync_state is SyncState.SynchronousRealTime:
                        _ = world.tick()

                    if sync_state is SyncState.SynchronousRealTime:
                        tick_manager_node.sleep_real_time()

                if simulation_start_node.check_timeout():
                    logging.info("  Mission Completed... Exceeded Time Limit")
                    break

            except:
                logging.info("  CARLA Client no longer connected, likely a system crash or the mission completed")
                raise Exception("CARLA Client no longer connected")
    except KeyboardInterrupt:
        logging.info("  System Shutdown Command, closing out System Manager")
    except Exception as error:
        logging.info("  Error: %s", error)
        logging.info("  " + traceback.format_exc())
        logging.info("  System Error, Check log, likely CARLA is not connected. See if CARLA is running.")
        
    finally:
        try:
            if clean_kill:
                logging.info("  Clean Kill enabled, End World...")
                if original_settings:
                    client.get_world().apply_settings(original_settings)
                    _ = world.tick()
                    client.load_world("EndingWorld")
                logging.info("  Game Instance Closed, exiting System Manager...")
            else:
                logging.info("  Reseting Game to original state...")
                for actor in tracked_actors:
                    actor.destroy()

                if original_settings:
                    client.load_world("StartingWorld")
                    client.get_world().apply_settings(original_settings)

                _ = world.tick()
                logging.info("  Game finished resetting, exiting System Manager...")
        except KeyboardInterrupt:
            print("main(): KeyboardInterrupt caught in [scenario-node]")
        except Exception as error:
            logging.info("  Error: %s", error)
            logging.info("  Failed to reset game to original state...")
            pass
    # Even though the daemon for the thread running the multithreaded executor is set to 'True',
    # shutting down the executor here ensures the thread terminates gracefully.
    executor.shutdown()

if __name__ == "__main__":
    main()