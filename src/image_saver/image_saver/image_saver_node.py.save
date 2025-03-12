import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time
import os

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.declare_parameter('save_interval', 5)  # interval in seconds to save the image
        self.save_interval = self.get_parameter('save_interval').get_parameter_value().integer_value
        self.bridge = CvBridge()

        # Replace with your camera's topic
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',  # Update this topic name to match your setup
            self.image_callback,
            10
        )

        # Timer to save an image every 5 seconds
        self.timer = self.create_timer(self.save_interval, self.timer_callback)
        
        self.last_saved_time = time.time()

    def image_callback(self, msg):
        """Callback to process the image and save it"""
        # Convert ROS image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return
        
        # Save the image to the download folder
        self.save_image(cv_image)

    def timer_callback(self):
        """Triggered every 5 seconds to save the latest captured image"""
        current_time = time.time()
        if current_time - self.last_saved_time >= self.save_interval:
            # Save the latest image (from callback)
            if hasattr(self, 'latest_image'):
                self.save_image(self.latest_image)

    def save_image(self, cv_image):
        """Save the captured image to the download folder"""
        # Path to save image
        download_path = os.path.expanduser('~/Downloads')
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        file_path = os.path.join(download_path, f"image_{timestamp}.png")

        cv2.imwrite(file_path, cv_image)
        self.get_logger().info(f"Image saved to {file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
