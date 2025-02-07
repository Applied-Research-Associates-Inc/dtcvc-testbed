# Using the DTC Testing Container to Drive the Quadrotor

When running the testbed, many nodes are included due to the nature of the competition. However, when working to integrate systems with a controllable robot, a simplified version of running the testbed has been included to aid in simulation development and robot integration. This approach still uses Docker Swarm but allows the user to run a container outside of the Swarm to test things without the need to fully integrate anything into the swarm.

**Note**: This step assumes that you have access to `dtcvc` and `DTC_TestingContainer` packages.

## Setup and Run Simulation

These steps detail how to modify and run the simulation.

### Modify the Scenario File or Change the Path/Name of the Scenario

If you wish to change things from the defaults, modify the scenario configuration found in `dtcvc/deployment/data/config/testbed/scenario.yaml` or the deployment configuration found in `dtcvc/deployment/swarm/solely-sim.yml`.

### Launch the Simulation Only Mode of the Virtual Testbed

To launch the simulation and inject a scenario, run the command below:

```bash
./deployment/scripts/swarm-deploy.sh ./swarm/solely-sim.yml
```

The scorekeeper should stop at:

```bash
[dtcvc_scenario_node-1] INFO:root:  Starting Simulation...
```

This means the simualtion has loaded a scenario and is paused waiting for you to send a start signal. Once you are here, you can move on to running the DTC Testing Container.

## Setup and Run DTC Testing Container

The DTC Testing Container is where things get less exact. It has some preset ways to run, but is meant more as an example interface and not a fully fleshed out package. This details running with RVIZ and running with ardupilot. Additionally, you can run these together if you like.

Build this package by running `docker compose build` then move onto the following steps as needed.

### Configure Docker Files to Work With Swarm (REQUIRED)

The docker compose file and the XML for the discovery server connection need to have the ip acquired earlier added to them.

In `compose_swarm.yaml`, change the ip on line 20 (10.0.1.2) to the one you acquired: 
`- ROS_DISCOVERY_SERVER=10.0.1.2:${DISCOVERY_SERVER_PORT:-11811}`

In `super_client_configuration_file.xml`, change the ip address on line 14 to the one you acquired.

### Run RVIZ (Optional)

With the scorekeeper returning:

```bash
[dtcvc_scenario_node-1] INFO:root:  Starting Simulation...
```

The simulation is in a state where it wants a start command published to it. Otherwise, it will hang indefinitely. To do this, you will need to send a start command manually. 

First, open an interactive docker container:

```bash
docker compose --file compose_swarm.yaml run --rm --entrypoint /bin/bash dtc_testing_container
```

Once in the docker, run the following to start the simulation:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /dtc/simulation_start std_msgs/msg/Empty {}\
```

Hit ctrl+c after a few of these publish and the terminal for the scorekeeper moves on and says it is running the simulation.

To visualize ROS messages coming from the simulation, you can use RVIZ by running in the terminal:

```bash
rviz2
```

### Run Ardupilot (Optional)

With the scorekeeper returning:

```bash
[dtcvc_scenario_node-1] INFO:root:  Starting Simulation...
```

The simulation is in a state where it wants a start command published to it. Otherwise, it will hang indefinitely. To do this, you just need to run Ardupilot. The entry script will publish the start prior to opening Ardupilot.

To open Ardupilot, all you need to do is run:

```bash
docker compose --file compose_swarm.yaml up 
```

This will open Ardupilot, from here you can follow the next section of instructions. However, if you also want to open Rviz, you can exec into this container in another terminal and run the next commands:

```bash
docker exec -it DTCTestingContainer /bin/bash
source /opt/ros/humble/setup.bash
rviz2
```

#### How to use Ardupilot

1. Once the Ardupilot opens:
    - After a few seconds you should see a white window pop up, followed by QGroundControl shortly after
    - If you do not see [Common Problems](#common-problems-qgroundcontrol-doesnt-open)
3. After a few seconds the QGroundControl window will appear. You will get and error about the radio, you can ignore this. It should look like the image below.
    - It may also open the map screen, and say "Not Ready" in the top left, this is also fine
    - If it says "Disconnected" in the top left see [Common Problems](#common-problems-qgroundcontrol-says-disconnected)
    ![Startup Image](/Images/QGroundControl.png)
4. Click the megaphone at the top of the window to open the QGC console output
    - This may also appear as a red and white triangle with an exclamation point inside if there has been an error or warning generated.
    - See [Common Problems](#common-problems-ardupilot-warnings) for a list of Ardupilot warnings
5. Wait until "EKF3 IMU0 is using GPS" is printed to the QGC console, this may take awhile (1-2 mins max) ![Ready Image](/Images/QGroundControl-ReadyToFly.png)
6. The quadcopter is ready to fly, close the QGC console if it's open, click the ready to fly text in the top right, and arm the vehicle
7. After arming the vehicle click the takeoff button in the top left and drag the slider. The vehicle should takeoff
8. click the map and click "Go to Location" to move the quadcopter to a waypoint

There is a video showing these steps in the videos folder of the DTC Testing Container


#### Common Problems

##### Common Problems: QGroundControl doesn't open  
1. Stuck on "Waiting for topic '/dtc/simulation_start' to exist..." / The container cannot find the `/dtc/simulation_start` topic
    - Make sure the simulation is running and has not crashed. You can inspect the logs using `docker service logs solely-sim_simulator1` and `docker service logs solely-sim_scorekeeper`
    - Make sure you have given sufficient time for the simulation to start and the scenario to be loaded. This may take up to 1 minute after running `./swarm-deploy.sh ./swarm/solely-sim.yml`
    - Make sure the `ROS_DISCOVERY_SERVER` ip address has been updated in compose_swarm.yaml and super_client_configuration_file.xml to be the ip address of the dtcvc-discovery. You can find this ip by running `docker service inspect dtcvc-discovery`

##### Common Problems: QGroundControl says Disconnected
- Make sure the simulation is running and has not crashed. You can inspect the logs using `docker service logs solely-sim_simulator1` and `docker service logs solely-sim_scorekeeper`
- Make sure you have given sufficient time for the simulation to start and the scenario to be loaded. This may take up to 1 minute after running `./swarm-deploy.sh ./swarm/solely-sim.yml`
- Make sure you are only one instance of the simulation
- Make sure the loaded scenario includes a quadcopter with an imu. The following ros2 topics must exist (dtc_vehicle refers to the name of the vehicle and may be different): 
    ```
    /carla/dtc_vehicle/imu
    /carla/dtc_vehicle/multirotor_control_cmd
    /carla/dtc_vehicle/odometry
    ```
    - **Ensure that data is being published to the imu and odometry topics** if there are not publishers for these topics the required sensors are likely missing from the scenario file

##### Common Problems: Ardupilot Warnings
- There are several warnings that may be generated prior to seeing "EKF3 IMU0 is using GPS" printed in the console. The following warnings may be ignored if they occur prior to "EKF3 IMU0 is using GPS"
    ```
    anything related to inconsistent data
    EKF attitude is bad
    Terrain: clamping offset
    not using configured AHRS type
    ```
- The following are known errors/warnings that occur while taking off and can be ignored so long as they do not prevent takeoff or occur repeatedly after taking off
    ```
    Critical: GPS Glitch or Compass error
    EKF3 IMU emergency yaw reset
    ```
