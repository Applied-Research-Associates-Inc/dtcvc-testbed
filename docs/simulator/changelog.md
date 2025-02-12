# DTC Simulation Changelog

Contains the changes to the 3D Sensor and Physic Unreal Package distributed for the DTC program. DTC Stable name is also the tags for the repositories used excluding `carla-content`. 

Repositories pertaining to `dtc_stable`:
- https://github.com/Applied-Research-Associates-Inc/DTC_TestingContainer
- https://github.com/Applied-Research-Associates-Inc/dtc_plugin
- https://github.com/Applied-Research-Associates-Inc/carla
- https://github.com/Applied-Research-Associates-Inc/carla_integration_plugins
- https://github.com/Applied-Research-Associates-Inc/dtcvc-testbed

With the final release maintained through https://github.com/Applied-Research-Associates-Inc/DTCRelease

## dtc_stable_2.0.1:
- Fixed most of the underrun audio issues
- Fixed foliage temperature in the IR camera 
- Fixed issues with the GPS sensor
- Fixed a number of collision issues
- Fixed crash issues with Train Crash map

## dtc_stable_2.0.0:
- Upgraded Carla and the DTC Simulation System to Unreal 5.5
- Redesigned Simulation Architecture
- Redesigned Scenario Architecture
- Included Robotics Control and Controllable Quadrotor
- Updated IR Sensor to be correct loadout and fixed a number of issues with the returns
- Added New Testing Environment with variants
- Added Audio Sensor
- Created DTC Testing Container to Work with robot platform
- Metahumans now look at the robot when it is near them
- Updated simulation and manager to allow for parallel robotics
- Include external trigger to allow for Multiverse simulations (pseudo server/client)
- Fully integrated into the Virtual Testbed with a simulation-only option

## dtc_stable_1.0:
- Fixed RGB and IR Camera
- Added Ground Truth Odometry Sensor
- Fixed Vehicle Movement
- Added Simple Audio Tracking System
- Fixed environmental assets
- Hooked up Carla Weather
- Updated missions to include all 60 casualties and 30 waypoints
- Added the Train Crash map
- Updated Robot configuration to include better parameters and a RADAR
- Fixed Movement Component
- Added Dwell Times to Waypoints

## dtc_stable_0.2:
- Added nodes to System Manager for starting the simulation and ending the simulation when the mission is complete.
- Added functionality to cleanly exit the simulation.

## dtc_stable_0.1:
- Generated first working Carla integrated build.
- Added configurable MetaHumans spawner.
- Added configurable Waypoint nodes.
- System runs waypoint mission with default sensor feeds.
- Added MetaHumans to LiDAR returns.
- Added System Manager script.

## dtc_stable_0.0:
- demo release
- static mission with sensor feeds integrated with Unreal