from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_srvs.srv import Empty as EmptySrv

class SyncState:
    Unknown = 0
    Synchronous = 1
    SynchronousRealTime = 2
    SynchronousExternalTrigger = 3

class TickManagerNode(Node):
    def __init__(self):
        super().__init__("tick_manager_node")
        self._carla_client = None
        self._carla_world = None
        self._sync_state = None
        self._loop_rate = None
        self._is_tick_paused = False
        self._sim_time_step = 0.05
        self._sync_state_dict = {0:"Unknown", 1:"Synchronous", 2:"SynchronousRealTime", 3:"SynchronousExternalTrigger"}
        self._pause_tick_srv = self.create_service(EmptySrv, '/pause_tick', self.pause_tick_cb)
        self._resume_tick_srv = self.create_service(EmptySrv, '/resume_tick', self.resume_tick_cb)
        self._trigger_frame_sub = self.create_subscription(Empty, "/external_trigger/trigger_frame", self.trigger_frame_cb, 10)
        self._frame_processed_pub = self.create_publisher(Bool, "/external_trigger/frame_processed", 10)
        self.get_logger().info("TickManagerNode started!")

    def pause_tick_cb(self, req, resp):
        self._is_tick_paused = True
        self.get_logger().info("Ticking paused!")
        return resp

    def resume_tick_cb(self, req, resp):
        self._is_tick_paused = False
        self.get_logger().info("Ticking resumed!")
        return resp

    def trigger_frame_cb(self, msg):
        if self._sync_state is SyncState.SynchronousExternalTrigger:
            if not self.is_tick_paused():
                self._carla_world.tick()
                self._frame_processed_pub.publish(Bool(data=True))
            else:
                self.get_logger().warn("Cannot trigger frame since ticking has been paused.")
        else:
            self.get_logger().warn("Cannot trigger frame since not in the Synchronous External Trigger state.")

    def set_sync_state(self, sync_state, sim_time_step=None):
        if self._carla_client is None:
            self.get_logger().warn("No carla client was set...")
            return False
        world = self._carla_client.get_world()
        settings = world.get_settings()
        settings.synchronous_mode = True
        if sim_time_step != 0 and sim_time_step is not None:
            settings.fixed_delta_seconds = sim_time_step
            self._sim_time_step = sim_time_step
            self.get_logger().info("Sim Time Step changed to %f seconds." % sim_time_step)
        world.apply_settings(settings)
        if sync_state is SyncState.SynchronousRealTime:
            self._loop_rate = self.create_rate(1/self._sim_time_step, self.get_clock())
        self._sync_state = sync_state
        self.get_logger().info("Sync State set to %s." % self._sync_state_dict[self._sync_state])
        return True

    def get_sync_state(self):
        return self._sync_state

    def get_sim_time_step(self):
        return self._sim_time_step
    
    def sleep_real_time(self):
        self._loop_rate.sleep()

    def is_tick_paused(self):
        return self._is_tick_paused

    def set_carla_client(self, carla_client):
        self._carla_client = carla_client
        self._carla_world = self._carla_client.get_world()