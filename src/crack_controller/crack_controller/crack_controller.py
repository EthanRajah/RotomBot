import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix

class FlightController(Node):
    def __init__(self):
        super().__init__('flight_controller')

        # **Drone State Variables**
        self.current_pose = None  # Current position of the drone
        self.current_waypoint = None  # Stored goal position
        self.hold_count = 0
        self.active = False  # Control active flag
        self.waypoints = None
        self.obs_centers = None # Dictionary of obstacle centers: {A:[x, y], B:[x, y], C:[x, y], D:[x, y]}
        # Pass in transform from map to vicon/world
        self.home = None
        # track if we set the first waypoint
        self.first_waypoint_set = False
        self.drone_radius = 0.25
        self.safety_radius = 0.15 + 2*self.drone_radius

        # **Subscribers**
        self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.pose_callback, 50)

        # **Publishers**
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # **Services**
        # self.create_response = self.create_service(Trigger, '/comm/test', self.handle_test)
        # self.land_response = self.stop_service(Trigger, '/comm/land', self.handle_land)

        # **Timer for Publishing Setpoints (20Hz)**
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Flight Controller Initialized")

    ## === CALLBACKS === ##
    def pose_callback(self, msg):
        """ Updates the drone's current position. """
        self.current_pose = msg.pose

    def check_within_obstacles(self, current_pose, obs_centers, safety_radius):
        """ Check if the current position of the drone is within any of the obstacle radius
        INPUT - current_pose: PoseStamped() object, obs_centers: Dictionary of obstacle centers: {A:[x, y], B:[x, y], C:[x, y], D:[x, y]}
                safety_radius: float
        OUTPUT - Repulsive vector: numpy array [x, y]. The vector from each obstacle center to the current position of the drone will be 
                weighted by the inverse of the distance between the drone and the obstacle center.
        """
        repulsive_vec = np.array([0, 0, 0])
        within_obs = {}
        for obs in obs_centers.keys():
            obs_center = obs_centers[obs]
            dist = np.linalg.norm([current_pose.position.x - obs_center[0], current_pose.position.y - obs_center[1]])
            if dist < safety_radius:
                repulsive_vec += np.array([current_pose.position.x - obs_center[0], current_pose.position.y - obs_center[1], 0]) / dist /max(0.25 , dist) 
                within_obs[obs] = obs_center
        return repulsive_vec, within_obs
        
    ## === CONTROL LOOP === ##
    def control_loop(self):
        """ Controls the drone by publishing waypoints. """

        # Process self.waypoints
        #self.get_logger().info(f"Waypoints: {self.waypoints}")
        if not self.first_waypoint_set:
            if self.current_waypoint == None and self.waypoints:
                self.get_logger().info("Setting current waypoint to first.")
                # Set current_waypoint to first. Only occurs when we first start TEST
                self.current_waypoint = self.waypoints.pop(0)
                self.first_waypoint_set = True
        #self.get_logger().info(f"active: {self.active}, curpose: {self.current_pose}, curwaypt: {self.current_waypoint}")
        if self.active and self.current_pose and self.current_waypoint:
            # Here, current_waypoint coordinates are in the initial frame., want coordinates in cube frame. do new transforms:
            # 1. Initial frame to cube frame (composed of matrix of /mavros/vision_pose/pose readings)
            
            # TODO: GURPREET EDIT FROM HERE ONWARDS
            #Check if the current position of the drone is within any of the obstacle radius
            repulsive_vec, obs_within = self.check_within_obstacles(self.current_pose, self.obs_centers, self.safety_radius)
            
            # Compute distance to current waypoint
            move_vector = np.array([
                self.current_pose.position.x - self.current_waypoint.position.x,
                self.current_pose.position.y - self.current_waypoint.position.y,
                self.current_pose.position.z - self.current_waypoint.position.z
            ])
            
            dist = np.linalg.norm(move_vector)

            # Create setpoint message
            sp = PoseStamped()
            sp.header.stamp = self.get_clock().now().to_msg()
            sp.header.frame_id = "map"

            if dist < 0.75:
                # # **If within 0.5m, publish the waypoint directly**
                # sp.pose.position  = self.current_waypoint.position
                # sp.pose.orientation = self.current_waypoint.orientation
                # # self.get_logger().info("Hovering at start position.")
                
                ######
                move_vector_augmented = move_vector + repulsive_vec
                move_vector_augmented = move_vector_augmented / np.linalg.norm(move_vector_augmented) * dist
                
                sp.pose.position.x = self.current_pose.position.x + move_vector_augmented[0]
                sp.pose.position.y = self.current_pose.position.y + move_vector_augmented[1]
                sp.pose.position.z = self.current_pose.position.z + move_vector_augmented[2]
                sp.pose.orientation = self.current_waypoint.orientation
            else:
                # **Move 0.5m closer to start**
                # move_vector = np.array([
                #     self.current_waypoint.position.x - self.current_pose.position.x,
                #     self.current_waypoint.position.y - self.current_pose.position.y,
                #     self.current_waypoint.position.z - self.current_pose.position.z
                # ])
                # move_vector = move_vector / np.linalg.norm(move_vector) * 0.75  # Normalize and scale
                
                move_vector_augmented = move_vector + repulsive_vec
                move_vector_augmented = move_vector_augmented / np.linalg.norm(move_vector_augmented) * 0.75
                
                sp.pose.position.x = self.current_pose.position.x + move_vector_augmented[0]
                sp.pose.position.y = self.current_pose.position.y + move_vector_augmented[1]
                sp.pose.position.z = self.current_pose.position.z + move_vector_augmented[2]
                sp.pose.orientation = self.current_waypoint.orientation

                # self.get_logger().info(f"Moving toward waypoint: {sp.pose.position}")

            # Use dist to see if we're within ball radius of waypoint
            ball_radius = 0.25
            if dist < ball_radius and self.hold_count < 20:
                self.hold_count += 1
            elif dist < ball_radius and len(self.waypoints):
                # Update current_waypoint if waypoints list is not empty
                self.get_logger().info(f"Waypoint reached: {self.current_waypoint}")
                self.current_waypoint = self.waypoints.pop(0)
                self.hold_count = 0
                self.get_logger().info(f"Next waypoint: {self.current_waypoint}")
            elif dist < ball_radius:
                # Send land command
                sp = PoseStamped()
                sp.header.stamp = self.get_clock().now().to_msg()
                sp.header.frame_id = "map"
                # if self.home:
                #     pose.pose.position.x = self.home[0]
                #     pose.pose.position.y = self.home[1]
                #     pose.pose.position.z = 0.15
                # else:
                sp.pose.position.x = self.current_waypoint.position.x
                sp.pose.position.y = self.current_waypoint.position.y
                sp.pose.position.z = 0.1
                sp.pose.orientation.w = 1.0 
                self.get_logger().info(f"Waypoints Complete. Landing!")
            # Publish the waypoint
            self.setpoint_pub.publish(sp)

def main(args=None):
    rclpy.init(args=args)
    node = FlightController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()