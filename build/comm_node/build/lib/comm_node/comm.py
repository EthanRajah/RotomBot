import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # Service message type for handling commands
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from exer2_control.controller import FlightController
from vicon_bridge.vicon_bridge import ViconBridge
from realsense2mavros.realsense2mavros import realsense2mavros
from rclpy.executors import MultiThreadedExecutor
import time


# Callback Handlers
def handle_launch():
    print('[INFO] Launch Requested: The drone should take off.')

def handle_test():
    print('[INFO] Test Requested: The drone should hover and wait for scoring.')

def handle_land():
    print('[INFO] Land Requested: The drone should descend for landing.')

def handle_abort():
    print('[INFO] Abort Requested: Emergency! The drone should land immediately.')

# Service Callbacks



def callback_abort(request, response):
    handle_abort()
    response.success = True
    response.message = "Abort command executed."
    return response

# Define the ROS 2 Node
class CommNode(Node):
    def __init__(self):
        super().__init__('comm_node')
        # HARDCODE Test swapper field
        self.test2 = True
        # Initialize publishers to/mavros/vision_pose/pose
        self.vicon = ViconBridge()
        self.realsense = realsense2mavros()
        # initialize the vicon to publish only (if test 1), else use vicon
        self.get_logger().info(str(self.test2))
        if self.test2:
            self.vicon.publish = False
            self.realsense.publish = True
            self.get_logger().info("Only publishing realsense to /mavros/vision_pose/pose!")
        else:
            self.vicon.publish = True
            self.realsense.publish = False
            self.get_logger().info("Only publishing vicon to /mavros/vision_pose/pose!")
        # self.realsense.publish = True
    
        # Initialize flight controller
        self.flight_controller = FlightController()

        self.test_enabled = False
        
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        self.target_altitude = 1.5  # meters
        
        # Create 20 Hz timer (1/20 = 0.05)
        self.timer = self.create_timer(0.05, self.publish_hover_setpoint)
        # Create Service Servers
        self.srv_launch = self.create_service(Trigger, '/rob498_drone_4/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, '/rob498_drone_4/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, '/rob498_drone_4/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, '/rob498_drone_4/comm/abort', self.callback_land)

        self.get_logger().info("CommNode started and ready to receive commands.")
        
        # MAVROS services for arming and mode switching
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # MAVROS publisher for position setpoints

        # Timer to send setpoints at a fixed rate

        # Keep track of activation status
        self.hover_active = True

        self.get_logger().info("DroneControlNode initialized. Ready to receive launch commands.")
        

    def arm_drone(self):
        """ Sends command to arm the drone """
        self.get_logger().info("Arming drone...")
        while not self.arming_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for /mavros/cmd/arming service...")

        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        #timeout = 2.0
        #start_time = time.time()
        #while time.time() - start_time < timeout:
        #    rclpy.spin_once(self, timeout_sec=0.1)
        '''
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().success:
            self.get_logger().info("Drone armed successfully!")
            return True
        else:
            self.get_logger().error("Arming failed!")
            return False
        '''

    def disarm_drone(self): 
        """ Sends command to arm the drone """
        self.get_logger().info("disarming drone...")
        while not self.arming_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for /mavros/cmd/arming service...")

        req = CommandBool.Request()
        req.value = False
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("Drone disarmed successfully!")
            return True
        else:
            self.get_logger().error("Arming failed!")
            return False

    def set_offboard_mode(self):
        """ Switches flight mode to OFFBOARD """
        self.get_logger().info("Switching to OFFBOARD mode...")
        while not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for /mavros/set_mode service...")

        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(req)
        '''
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().mode_sent:
            self.get_logger().info("OFFBOARD mode enabled!")
            return True
        else:
            self.get_logger().error("Failed to set OFFBOARD mode!")
            return False
        '''
    def publish_hover_setpoint(self):
        """ Publishes position setpoints to maintain a hover """
        if not self.hover_active:
            return

        # self.target_altitude = 2.0  # meters
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = self.target_altitude
        pose.pose.orientation.w = 1.0  # Keep stable orientation

        self.pose_pub.publish(pose)
        # self.get_logger().info("Publishing hover setpoint...")

    def callback_launch(self, request, response):
        
        # handle_launch(response)
        
        self.get_logger().info("Launch command received!")

        # Arm the drone
        self.arm_drone()
        '''
        if not self.arm_drone():
            response.success = False
            response.message = "Failed to arm the drone."
            return response
        '''
        self.get_logger().info("Drone armed successfully.")    
    
        # Switch to OFFBOARD mode
        self.set_offboard_mode()
        '''
        if not self.set_offboard_mode():
            response.success = False
            response.message = "Failed to switch to OFFBOARD mode."
            return response
        '''
        # Activate hover mode
        self.hover_active = True
        self.get_logger().info(f"Drone is taking off to {self.target_altitude}m and hovering.")

        response.success = True
        response.message = "Launch command executed successfully."
        return response
    
    def callback_test(self, request, response):
        
        self.get_logger().info("Test command received!")
        
        self.test_enabled = True
        self.flight_controller.active = True
        self.flight_controller.handle_test(response)
        self.hover_active = False

        # if test 2 (no vicon for control), publish the realsense bridge to /mavros/vision_pose/pose instead of the vicon bridge after getting
        # initial readings from vicon
        # self.get_logger().info(str(self.test2))
        # if self.test2:
        #     self.vicon.publish = False
        #     self.realsense.publish = True
        #     self.get_logger().info("Swapped from vicon data to realsense!")
        response.success = True
        response.message = "Test command executed."
        return response
    
    def callback_land(self, request, response):
        self.flight_controller.active = False
        self.target_altitude = 0.15
        self.hover_active = True
        response.success = True
        self.get_logger().info("Land command received!")
        response.message = "Land command executed."
        return response

def main(args=None):
    # rclpy.init(args=args)
    # node = CommNode()
    # rclpy.spin(node)
    # rclpy.spin(node.realsense)
    # rclpy.spin(node.vicon)
    # rclpy.shutdown()
    
    rclpy.init(args=args)
    comm_node = CommNode()
    # Assuming both realsense and vicon are attributes added in CommNode
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(comm_node)
    executor.add_node(comm_node.realsense)
    executor.add_node(comm_node.vicon)
    executor.add_node(comm_node.flight_controller)

    try:
        executor.spin()
    finally:
        comm_node.destroy_node()
        comm_node.realsense.destroy_node()
        comm_node.vicon.destroy_node()
        comm_node.flight_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()