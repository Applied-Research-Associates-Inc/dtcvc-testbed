# DTC-VC Testbed ROS2 Packages

These are the ROS2 packages used in the DTC-VC Testbed (located under `ros/src`):
- `dtcvc_scenario`: Configures Carla with the scenario laydown
- `dtcvc_scorekeeper_types`: ROS2 message type definitions used by the scorekeeper
- `dtcvc_scorekeeper`: Collects and scores triage reports
- `dtcvc_recorder`: Manages recording and playback of messages
- `dtcvc_timekeeper`: Manages the competition clock, waiting for other services to be ready, and sending competition start and stop signals 