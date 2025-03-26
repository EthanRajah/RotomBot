import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import time

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=%d, height=%d, framerate=%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, format=BGRx ! videoconvert ! "
        "video/x-raw, format=BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
        )
    )

class CameraSaverNode(Node):
    def __init__(self):
        super().__init__('camera_saver')

        # ROS Publisher
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # OpenCV Camera Capture
        self.cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera. Check pipeline.")

        # Timers
        self.timer_pub = self.create_timer(1.0 / 30, self.capture_frame)  # 30 FPS
        self.timer_save = self.create_timer(5.0, self.save_image)  # Save every 5 sec

        self.latest_frame = None  # Store latest frame

    def capture_frame(self):
        """Capture frame, publish it, and store the latest frame."""
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            self.latest_frame = frame
            self.get_logger().info("Published frame")
        else:
            self.get_logger().warn("Failed to capture frame")

    def save_image(self):
        """Save the latest captured frame every 5 seconds."""
        if self.latest_frame is not None:
            download_path = os.path.expanduser('~/Downloads')
            os.makedirs(download_path, exist_ok=True)
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            file_path = os.path.join(download_path, f"image_{timestamp}.png")

            cv2.imwrite(file_path, self.latest_frame)
            self.get_logger().info(f"Image saved to {file_path}")

    def destroy_node(self):
        """Release the camera before shutting down the node."""
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
