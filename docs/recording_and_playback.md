# Recording and Playback

## 2 - Recorder

The <b>dtcvc_recorder_node</b> node can be set to automatically start recording the scenario simulation as soon as the containers are brought up by setting the 'record_on_startup' parameter to 'True'. Otherwise the '~/start_recording' service can be used. Stopping the recorder is done automatically once the containers are taken down. Otherwise the '~/stop_recording' service can be used.

Recordings are saved as '.bag' folders which should contain a metadata file and an mcap file. These '.bag' folders can be found in the '<root_directory>/dtcvc/recordings' directory.

### 2.1 - Calling a service

- <code>source /setup.bash</code>
- <code>ros2 service call <service_name> <service_message_type></code>

### 2.2 - List of Services

#### 2.2.1- Format:
- <service_name>
  - <service_message_type>
  - <service_description>

#### 2.2.2 - Services:
- /dtcvc_recorder_node/start_recording 
  - `std_srvs/srv/Trigger`
  - Begins the recording process.
- /dtcvc_recorder_node/stop_recording 
  - `std_srvs/srv/Trigger`
  - Stops the recording process.
- /dtcvc_recorder_node/is_paused 
  - `rosbag2_interfaces/srv/IsPaused`
  - Returns whether recording is currently paused.
- /dtcvc_recorder_node/pause 
  - `rosbag2_interfaces/srv/Pause`
  - Pauses recording. All messages that have already arrived will be written, but all messages that arrive after pausing will be discarded. Has no effect if already paused. Takes no arguments.
- /dtcvc_recorder_node/resume 
  - `rosbag2_interfaces/srv/Resume`
  - Resume recording if paused. Has no effect if not paused. Takes no arguments.
- /dtcvc_recorder_node/split_bagfile 
  - `rosbag2_interfaces/srv/SplitBagfile`
  - Triggers a split to a new file, even if none of the configured split criteria (such as --max-bag-size or --max-bag-duration) have been met yet
- /dtcvc_recorder_node/snapshot 
  - `rosbag2_interfaces/srv/Snapshot`
  - enabled if --snapshot-mode is specified. Takes no arguments, triggers a snapshot.

## 3 - Playback

The <b>dtcvc_playback_node</b> is responsible for playing back a recorded scenario for more effecient competition simulation. It is automatically setup for you in the "dtcvc_scorekeeper_no_sim.launch.py" and all that needs to be done is calling the "~/start_playback" service. These pre-made scenario recordings can be found int '<root_directory>/dtcvc/scenario_recordings'

### 3.1 - Calling a service

- <code>source /setup.bash</code>
- <code>ros2 service call <service_name> <service_message_type></code>

### 3.2 - List of Services

#### 3.2.1 - Format:
- <service_name>
  - <service_message_type>
  - <service_description>

#### 3.2.2 - Services:
- /dtcvc_recorder_node/burst 
  - `rosbag2_interfaces/srv/Burst`
  - Can only be used while player is paused, publishes num_messages in order as fast as possible, moving forward the play head.
- /dtcvc_recorder_node/get_rate 
  - `rosbag2_interfaces/srv/GetRate`
  - Return the current playback rate.
- /dtcvc_recorder_node/is_paused 
  - `rosbag2_interfaces/srv/IsPaused`
  - Return whether playback is paused.
- /dtcvc_recorder_node/pause 
  - `rosbag2_interfaces/srv/Pause`
  - Pause playback. Has no effect if already paused.
- /dtcvc_recorder_node/play 
  - `rosbag2_interfaces/srv/Play`
  - Play from a starting offset timestamp, either until the end, an ending timestamp or for a set duration. Only works when stopped (not paused).
- /dtcvc_recorder_node/play_next 
  - `rosbag2_interfaces/srv/PlayNext`
  - Play a single next message from the bag. Only works while paused.
- /dtcvc_recorder_node/resume 
  - `rosbag2_interfaces/srv/Resume`
  - Resume playback if paused.
- /dtcvc_recorder_node/seek 
  - `rosbag2_interfaces/srv/Seek`
  - Change the play head to the specified timestamp. Can be forward or backward in time, the next played message is the next immediately after the seeked timestamp.
- /dtcvc_recorder_node/set_rate 
  - `rosbag2_interfaces/srv/SetRate`
  - Sets the rate of playback, for example 2.0 will play messages twice as fast.
- /dtcvc_recorder_node/stop 
  - `rosbag2_interfaces/srv/Stop`
  - Stop the player, putting the play head in "undefined position" outside the bag. Must call play before other operations can be done.
- /dtcvc_recorder_node/toggle_paused 
  - `rosbag2_interfaces/srv/TogglePaused`
  - Pause if playing, resume if paused.