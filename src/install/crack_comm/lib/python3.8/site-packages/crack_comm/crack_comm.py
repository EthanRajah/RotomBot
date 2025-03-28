import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # Service message type for handling commands
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry  # use Odometry for reading directly from camera since it has both pose and twist
from geometry_msgs.msg import PoseStamped, PoseArray
from exer3_control.flight_controller import FlightController
from vicon_bridge.vicon_bridge import ViconBridge
from realsense2mavros.realsense2mavros import realsense2mavros
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
import time
import numpy as np

STATE = 'Init'
WAYPOINTS = None
WAYPOINTS_RECEIVED = False

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
        self.test2 = False
        self.get_logger().info("Test2 flag: " + str(self.test2))
        # Initialize publishers to/mavros/vision_pose/pose
        self.vicon = ViconBridge()
        self.realsense = realsense2mavros()
        # initialize the vicon to publish only (if test 1), else use vicon
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
        self.sub_waypoints = self.create_subscription(PoseArray, 'rob498_drone_4/comm/waypoints', self.callback_waypoints, 10)
        self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.pose_callback, 50)
        
        self.target_altitude = 0.5  # meters
        
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

        # Keep track of activation status
        self.hover_active = True

        # Store local position from whereever the drone starts
        self.current_pose = None
        # Initialize local start to 0 and 0 orientation
        self.local_start = PoseStamped()
        self.local_start.header.frame_id = "map"
        self.local_start.pose.position.x = 0.0
        self.local_start.pose.position.y = 0.0
        self.local_start.pose.position.z = 0.0

        self.local_start.pose.orientation.x = 0.0
        self.local_start.pose.orientation.y = 0.0
        self.local_start.pose.orientation.z = 0.0
        self.local_start.pose.orientation.w = 1.0  # Identity quaternion (no rotation)

        self.get_logger().info("DroneControlNode initialized. Ready to receive launch commands.")
        
        # Subscribe to '/vicon/ROB498_Drone/ROB498_Drone' for finding the transformation between vicon and realsense
        self.raw_vicon = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Drone/ROB498_Drone',
            self.vicon_callback,
            10)
        self.first_vicon = None
        # # Also subscribe to the camera pose to get initial camera pose for frame transform: NOW UNUSED
        # self.realsense_sub = self.create_subscription(
        #     Odometry,
        #     '/camera/pose/sample',
        #     self.realsense_callback,
        #     rclpy.qos.qos_profile_system_default)
        # self.first_realsense = None
        # Defines transformation matrix from vicon world frame to cube (map) frame
        self.T_2_0 = None

        # Final project: 4 subscribers for 4 obstacles (need center points, whereas api_waypoints_node sends goal drone poses)
        self.obs_A = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Obstacle_A/ROB498_Obstacle_A',
            self.obs_callback_A,
            10)
        self.obs_B = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Obstacle_B/ROB498_Obstacle_B',
            self.obs_callback_B,
            10)
        self.obs_C = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Obstacle_C/ROB498_Obstacle_C',
            self.obs_callback_C,
            10)
        self.obs_D = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Obstacle_D/ROB498_Obstacle_D',
            self.obs_callback_D,
            10)
        self.obs_centers = {}

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
        pose.pose.position.x = self.local_start.pose.position.x
        pose.pose.position.y = self.local_start.pose.position.y
        pose.pose.position.z = self.local_start.pose.position.z + self.target_altitude
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
        self.local_start = self.current_pose

        response.success = True
        response.message = "Launch command executed successfully."
        return response
    
    def callback_test(self, request, response):
        
        self.get_logger().info("Test command received!")
        
        self.test_enabled = True
        self.flight_controller.active = True
        # self.flight_controller.handle_test(response)
        self.hover_active = False

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
        
    def callback_waypoints(self, msg):
        '''Waypoints given in one shot from service'''
        global WAYPOINTS_RECEIVED, WAYPOINTS
        if WAYPOINTS_RECEIVED:
            return
        print('Waypoints Received')
        WAYPOINTS_RECEIVED = True
        WAYPOINTS = np.empty((0,3))
        for pose in msg.poses:
            pos = np.array([pose.position.x, pose.position.y, pose.position.z])
            WAYPOINTS = np.vstack((WAYPOINTS, pos))
        # Transform points into cube frame before we send to flight controller
        waypoints_transformed = self.find_transformation()
        # Get waypoints in list format
        waypoints_list = list(waypoints_transformed)
        self.get_logger().info(f"transformed waypoints: {waypoints_list}")
        # Convert each waypoint to Pose message
        waypoints_pose_list = []
        for waypoint in waypoints_list:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            # Append the pose only
            waypoints_pose_list.append(pose.pose)
        # Add home waypoint to the end
        waypoints_pose_list.append(self.local_start.pose)
        # Set flight controller waypoints list to be waypoints array gotten here
        self.flight_controller.waypoints = waypoints_pose_list
        self.flight_controller.obs_centers = self.obs_centers

    def pose_callback(self, msg):
        """ Updates the drone's current PoseStamped message. """
        self.current_pose = msg

    def find_transformation(self):
        if not self.test2:
            # Test 1: Pure vicon: Simply apply vicon -> cube transformation and change frame id to map
            transform = self.vicon.transform
            waypoints_transformed = WAYPOINTS + transform # Should broadcast: (N, 3) + (3)
            return waypoints_transformed
            
        else:
            # Test 2: Realsense only: Get transformation that transforms vicon to realsense, then realsense to cube (cube), then apply this transform to each waypoint
            if self.first_vicon is not None:
                # T_1^0 = [C(q), t]
                t, q = self.first_vicon
                t = [t.x, t.y, t.z]
                q = [q.x, q.y, q.z, q.w]
                T_1_0 = quaternion_matrix(q)
                T_1_0[:3, 3] = np.array(t).reshape(3)

                # T_2_1: correct marker to realsense transformation
                marker_to_realsense = np.array([0.089, 0, 0.012]) - np.array([-0.124, 0, 0.103])
                T_2_1 = np.eye(4)
                T_2_1[:3, 3] = marker_to_realsense
                T_2_0 = T_1_0 @ T_2_1

                # Transform realsense readings to vicon world frame: T_0_2
                self.T_2_0 = T_2_0
                self.realsense.T_2_0 = T_2_0
                self.get_logger().info(f"Transformed T_2_0: {T_2_0}. Test2 is active")
                return WAYPOINTS
            else:
                self.get_logger().warn("No vicon or realsense data received yet! You really fucked something up!")
    
    def vicon_callback(self, msg):
        # Create new PoseStamped message for MAVROS
        mavros_msg = PoseStamped()

        # Copy header
        mavros_msg.header = msg.header
        mavros_msg.header.frame_id = 'map'

        transform = self.vicon.transform
        # Copy vicon position data and transform to cube frame
        mavros_msg.pose.position = msg.pose.position
        mavros_msg.pose.position.x += transform[0]
        mavros_msg.pose.position.y += transform[1]
        mavros_msg.pose.position.z += transform[2]

        # Copy orientation data
        mavros_msg.pose.orientation = msg.pose.orientation

        # Save pos and orientation tuple only once
        if self.first_vicon is None:
            self.get_logger().info('First vicon received and logged in waypoint.py')
            self.first_vicon = (mavros_msg.pose.position, mavros_msg.pose.orientation)

    def obs_callback_A(self, msg):
        # Create new PoseStamped message for MAVROS
        mavros_msg = PoseStamped()

        # Copy header
        mavros_msg.header = msg.header
        mavros_msg.header.frame_id = 'map'

        # Copy vicon position data 
        mavros_msg.pose.position = msg.pose.position

        # Copy x,y coordinates to obs_centers at key
        self.obs_centers['A'] = (mavros_msg.pose.position.x, mavros_msg.pose.position.y)

    def obs_callback_B(self, msg):
        # Create new PoseStamped message for MAVROS
        mavros_msg = PoseStamped()

        # Copy header
        mavros_msg.header = msg.header
        mavros_msg.header.frame_id = 'map'

        # Copy vicon position data 
        mavros_msg.pose.position = msg.pose.position

        # Copy x,y coordinates to obs_centers at key
        self.obs_centers['B'] = (mavros_msg.pose.position.x, mavros_msg.pose.position.y)

    def obs_callback_C(self, msg):
        # Create new PoseStamped message for MAVROS
        mavros_msg = PoseStamped()

        # Copy header
        mavros_msg.header = msg.header
        mavros_msg.header.frame_id = 'map'

        # Copy vicon position data 
        mavros_msg.pose.position = msg.pose.position

        # Copy x,y coordinates to obs_centers at key
        self.obs_centers['C'] = (mavros_msg.pose.position.x, mavros_msg.pose.position.y)

    def obs_callback_D(self, msg):
        # Create new PoseStamped message for MAVROS
        mavros_msg = PoseStamped()

        # Copy header
        mavros_msg.header = msg.header
        mavros_msg.header.frame_id = 'map'

        # Copy vicon position data 
        mavros_msg.pose.position = msg.pose.position

        # Copy x,y coordinates to obs_centers at key
        self.obs_centers['D'] = (mavros_msg.pose.position.x, mavros_msg.pose.position.y)

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