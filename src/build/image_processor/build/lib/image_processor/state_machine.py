import rclpy
from rclpy.node import Node
import threading

class DroneWaypointProcessor(Node):
    def __init__(self):
        super().__init__('drone_waypoint_processor')

        # --- Parameters and State ---
        self.current_state = 'IDLE'
        self.last_waypoint = None
        self.lock = threading.Lock()

        # --- Timer for checking state ---
        self.timer = self.create_timer(1.0, self.state_machine_loop)

    def start_pipeline(self, waypoint_id):
        with self.lock:
            if self.current_state == 'IDLE' and waypoint_id != self.last_waypoint:
                self.last_waypoint = waypoint_id
                self.get_logger().info(f"Starting pipeline for waypoint: {waypoint_id}")
                self.current_state = 'CAPTURING'
            else:
                self.get_logger().info(f"Waypoint {waypoint_id} ignored: already processing or duplicate.")

    def state_machine_loop(self):
        with self.lock:
            if self.current_state == 'CAPTURING':
                self.handle_capturing()
            elif self.current_state == 'SELECTING_IMAGE':
                self.handle_selecting_image()
            elif self.current_state == 'INFERENCING':
                self.handle_inferencing()
            elif self.current_state == 'SENDING':
                self.handle_sending()
            elif self.current_state == 'DONE':
                self.get_logger().info("Processing complete. Waiting for next waypoint...")
                self.current_state = 'IDLE'
            elif self.current_state == 'ERROR':
                self.get_logger().error("An error occurred. Resetting to IDLE.")
                self.current_state = 'IDLE'

    def handle_capturing(self):
        try:
            self.get_logger().info("Capturing images...")
            # --- Your image capture code here ---

            self.current_state = 'SELECTING_IMAGE'
        except Exception as e:
            self.get_logger().error(f"Capture failed: {e}")
            self.current_state = 'ERROR'

    def handle_selecting_image(self):
        try:
            self.get_logger().info("Selecting sharpest image...")
            # --- Your Laplacian image selection code here ---

            self.current_state = 'INFERENCING'
        except Exception as e:
            self.get_logger().error(f"Image selection failed: {e}")
            self.current_state = 'ERROR'

    def handle_inferencing(self):
        try:
            self.get_logger().info("Running inference...")
            # --- Your inference code here ---

            self.current_state = 'SENDING'
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            self.current_state = 'ERROR'

    def handle_sending(self):
        try:
            self.get_logger().info("Sending result to API...")
            # --- Your API call code here ---

            self.current_state = 'DONE'
        except Exception as e:
            self.get_logger().error(f"API call failed: {e}")
            self.current_state = 'ERROR'


def main(args=None):
    rclpy.init(args=args)
    node = DroneWaypointProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()