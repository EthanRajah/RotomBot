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
        self.start_position = None  # Stored goal position
        self.active = False  # Control active flag

        # **Subscribers**
        self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.pose_callback, 50)

        # **Publishers**
        # self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

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

    ## === SERVICE HANDLERS === ##
    def handle_test(self, request, response):
        """ Handles /comm/test - Starts the controller. """
        if not self.active:
            # First run when TEST service is activated but active flag is set false
            if self.current_pose:
                # Store goal pose as reference signal
                self.start_position = self.current_pose.position # Store the goal position
                self.start_orientation = self.current_pose.orientation # Store the goal orientation
                self.active = True
                self.get_logger().info(f"Test started. Storing start position: {self.start_position}")
                response.success = True
                response.message = "Test started"
            else:
                response.success = False
                response.message = "No valid pose data received yet."
        return response

    def handle_land(self, request, response):
        """Handle /comm/land - stop the controller"""
        if self.active:
            # Called when the TEST service is stopped, this will run
            self.active = False
            self.get_logger().info("Test stopped.")
            response.success = True
            response.message = "Test stopped"
        return response

    ## === CONTROL LOOP === ##
    def control_loop(self):
        """ Controls the drone by publishing waypoints. """
        if self.active and self.current_pose and self.start_position:
            # Compute distance to start position
            dist = np.linalg.norm([
                self.current_pose.x - self.start_position.x,
                self.current_pose.y - self.start_position.y,
                self.current_pose.z - self.start_position.z
            ])

            # Create setpoint message
            sp = PoseStamped()
            sp.header.stamp = self.get_clock().now().to_msg()
            sp.header.frame_id = "map"

            if dist < 0.5:
                # **If within 0.5m, hover at the start position**
                sp.pose.position = self.start_position
                self.get_logger().info("Hovering at start position.")
            else:
                # **Move 0.5m closer to start**
                move_vector = np.array([
                    self.start_position.x - self.current_pose.x,
                    self.start_position.y - self.current_pose.y,
                    self.start_position.z - self.current_pose.z
                ])
                move_vector = move_vector / np.linalg.norm(move_vector) * 0.5  # Normalize and scale

                sp.pose.position.x = self.current_pose.x + move_vector[0]
                sp.pose.position.y = self.current_pose.y + move_vector[1]
                sp.pose.position.z = self.current_pose.z + move_vector[2]
                sp.pose.orientation = self.start_orientation

                self.get_logger().info(f"Moving toward start: {sp.pose.position}")

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