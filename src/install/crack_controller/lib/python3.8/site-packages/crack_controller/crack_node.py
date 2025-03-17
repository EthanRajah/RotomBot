# STARTER CODE: Taken from RotomBot's flight_controller.py for exercise 3.
# TODO: when we get to each waypoint, hover for a few seconds until we get a good image, run onboard inference 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import numpy as np

class FlightController(Node):
    def __init__(self):
        super().__init__('flight_controller')

        # **Drone State Variables**
        self.current_pose = None  # Current position of the drone
        self.current_waypoint = None  # Stored goal position
        self.hold_count = 0
        self.active = False  # Control active flag
        self.waypoints = None
        # Pass in transform from map to vicon/world
        self.home = None
        # track if we set the first waypoint
        self.first_waypoint_set = False

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
            # Compute distance to current waypoint
            dist = np.linalg.norm([
                self.current_pose.position.x - self.current_waypoint.position.x,
                self.current_pose.position.y - self.current_waypoint.position.y,
                self.current_pose.position.z - self.current_waypoint.position.z
            ])

            # Create setpoint message
            sp = PoseStamped()
            sp.header.stamp = self.get_clock().now().to_msg()
            sp.header.frame_id = "map"

            if dist < 0.75:
                # **If within 0.5m, publish the waypoint directly**
                sp.pose.position = self.current_waypoint.position
                sp.pose.orientation = self.current_waypoint.orientation
                # self.get_logger().info("Hovering at start position.")
            else:
                # **Move 0.5m closer to start**
                move_vector = np.array([
                    self.current_waypoint.position.x - self.current_pose.position.x,
                    self.current_waypoint.position.y - self.current_pose.position.y,
                    self.current_waypoint.position.z - self.current_pose.position.z
                ])
                move_vector = move_vector / np.linalg.norm(move_vector) * 0.75  # Normalize and scale

                sp.pose.position.x = self.current_pose.position.x + move_vector[0]
                sp.pose.position.y = self.current_pose.position.y + move_vector[1]
                sp.pose.position.z = self.current_pose.position.z + move_vector[2]
                sp.pose.orientation = self.current_waypoint.orientation

                # self.get_logger().info(f"Moving toward waypoint: {sp.pose.position}")

            # Publish the waypoint
            self.setpoint_pub.publish(sp)

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

def main(args=None):
    rclpy.init(args=args)
    node = FlightController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()