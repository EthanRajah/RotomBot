#!usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import requests
import base64
from std_srvs.srv import Trigger
from .darknet_dir.inference_darknet import Darknet


class DroneWaypointProcessor(Node):
    def __init__(self):
        super().__init__('drone_waypoint_processor')
        self.get_logger().error("LightNet")

        # --- Parameters and State ---
        self.current_state = 'IDLE'
        self.last_waypoint = None
        self.lock = threading.Lock()

        # --- Timer for checking state ---
        self.timer = self.create_timer(1.0, self.state_machine_loop)

        # for camera: Timer to capture frames from cam at 30 fps
        self.timer_pub = self.create_timer(1.0 / 30, self.capture_frame)  # 30 FPS
        # OpenCV Camera Capture
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera. Check pipeline.")
        # Camera intrinsic params
        self.dist = np.array([-0.19927273, 0.0216427, 0.00122196, -0.00259398, 0.01506653]).reshape(-1, 1)
        self.newcameramtx = np.array([[8.07071504e+02, 0.00000000e+00, 1.04769754e+03],
                               [0.00000000e+00, 8.11702513e+02, 3.34017608e+02],
                               [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]).reshape(3, 3)
        self.mtx = np.array([[1.00914351e+03, 0.00000000e+00, 1.05808703e+03],
                            [0.00000000e+00, 1.01188475e+03, 3.33138874e+02],
                            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]).reshape(3, 3)
        self.roi = (35, 60, 1852, 956)
        # Store latest frame
        self.latest_frame = None
        # Define field for counting numbr of images processed for this waypoint
        self.image_count = 0
        self.desired_count = 5

        # For laplacian best image saving: store best image to be passed to CNN
        self.cur_best_image = None

        # For inference: self.load_net()
        config_file = '/home/jetson/ros2_ws/src/image_processor/image_processor/darknet_dir/training/yolov4-tiny-custom.cfg'
        weights_file = '/home/jetson/ros2_ws/src/image_processor/image_processor/darknet_dir/training/yolov4-tiny-custom_1000.weights'
        data_file = '/home/jetson/ros2_ws/src/image_processor/image_processor/darknet_dir/training/obj.data'
        self.net = Darknet(config_file, weights_file, data_file, threshold=0.1)
        self.confidence = None

        # For sending
        self.SERVER = "http://10.42.0.104:5072"
        self.IMAGE_MAP = {}

        # Client to send end_processing service call to comm node
        self.end_processing_client = self.create_client(Trigger, '/rob498_drone_4/comm/stop_image_processing')

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
                self.call_stop_image_processing_service(self.end_processing_client, 'Stop Image Processing')
                self.get_logger().info("Processing complete. Waiting for next waypoint...")
                self.current_state = 'IDLE'
            elif self.current_state == 'ERROR':
                self.get_logger().error("An error occurred. Resetting to IDLE.")
                self.current_state = 'IDLE'

    def handle_capturing(self):
        try:
            self.get_logger().info("Capturing images...")
            # --- Your image capture code here ---
            if self.image_count < self.desired_count:
                self.save_image()
                self.image_count += 1
                return
            self.image_count = 0
            self.current_state = 'SELECTING_IMAGE'
        except Exception as e:
            self.get_logger().error(f"Capture failed: {e}")
            self.current_state = 'ERROR'

    def handle_selecting_image(self):
        try:
            self.get_logger().info("Selecting sharpest image...")
            # --- Your Laplacian image selection code here ---
            folder_path = f'/home/jetson/Downloads/{str(self.last_waypoint)}'
            best_filename, best_image = self.find_sharpest_image(folder_path)
            best_filename = f'{folder_path}/{best_filename}'
            self.cur_best_image = best_image # (h, w, c)
            self.IMAGE_MAP[self.last_waypoint] = best_filename
            self.current_state = 'INFERENCING'
        except Exception as e:
            self.get_logger().error(f"Image selection failed: {e}")
            self.current_state = 'ERROR'

    def handle_inferencing(self):
        try:
            self.get_logger().info("Running inference...")
            # --- Your inference code here ---
            self.run_inference() # write true/false into self.confidence
            self.current_state = 'SENDING'
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            self.current_state = 'ERROR'

    def handle_sending(self):
        try:
            self.get_logger().info("Sending result to API...")
            # --- Your API call code here ---
            # if self.confidence:
            #     self.send_to_db()
            # else:
            #     self.get_logger().info(f"No crack detected at waypoint {self.last_waypoint}")
            self.send_to_db()
            self.current_state = 'DONE'
        except Exception as e:
            self.get_logger().error(f"API call failed: {e}")
            self.current_state = 'ERROR'

    ####### CAPTURING Helpers ##########

    def capture_frame(self):
        """Capture frame, publish it, and store the latest frame."""
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS Image message
            #msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            #self.publisher_.publish(msg)
            self.latest_frame = frame
            #self.get_logger().info("Published frame")
        else:
            self.get_logger().warn("Failed to capture frame")

    def save_image(self):
        """Save the latest captured frame every 5 seconds."""
        if self.latest_frame is not None:
            download_path = os.path.expanduser(f'~/Downloads/{str(self.last_waypoint)}')
            os.makedirs(download_path, exist_ok=True)
            timestamp = time.strftime("%Y%m%d-%H%M%S") + "_" + str(self.image_count)
            file_path = os.path.join(download_path, f"image_{timestamp}.png")

            # Apply calibrated intrinsics
            dst = cv2.undistort(self.latest_frame, self.mtx, self.dist, None, self.newcameramtx)
            x, y, w, h = self.roi
            dst = dst[y:y+h, x:x+w]
            cv2.imwrite(file_path, dst)
            self.get_logger().info(f"Image saved to {file_path}")

    # For camera saving
    def gstreamer_pipeline(
        self,
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

    ####### SELECTING_IMAGE Helpers ##########

    def laplacian_variance(self, image):
        """Compute the variance of the Laplacian of an image."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return cv2.Laplacian(gray, cv2.CV_64F).var()


    def find_sharpest_image(self, folder_path):
        """Find the sharpest image in a folder based on Laplacian variance."""
        best_image = None
        best_variance = 0
        best_filename = ""

        for filename in os.listdir(folder_path):
            if filename.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".tiff")):
                image_path = os.path.join(folder_path, filename)
                image = cv2.imread(image_path)

                if image is None:
                    continue

                variance = self.laplacian_variance(image)
                print(f"{filename}: Variance = {variance}")

                if variance > best_variance:
                    best_variance = variance
                    best_image = image
                    best_filename = filename

        print(f"Sharpest Image: {best_filename} with variance {best_variance}")
        return best_filename, best_image
    
    ####### Inference Helpers ##########

    
    def run_inference(self):
        # Image for inference is in self.cur_best_image (h, w, c)
        # NOTE: write True/False based on result in self.confidence
        # for debug: write true to confidence
        # print(f"type: {type(self.cur_best_image)}, shape: {self.cur_best_image.shape}, image: {self.cur_best_image}")
        result, save_path = self.net.run_inference(self.cur_best_image, process_red_box = True, image_save_path='/home/jetson/Downloads/testinference.jpeg')
        
        if save_path is not None:
            download_path = os.path.expanduser(f'~/Downloads/{str(self.last_waypoint)}')
            os.makedirs(download_path, exist_ok=True)
            timestamp = time.strftime("%Y%m%d-%H%M%S") + "_" + str(self.image_count)
            # Write T/F into the image name
            file_path = os.path.join(download_path, f"testinference_image_{timestamp}.png")
            cv2.imwrite(file_path, save_path)

        self.get_logger().info(f"Result-{result}")
        self.get_logger().info(f"Saved at-{save_path}")
        self.confidence = result

    ####### Sending Helpers #########
    def send_to_db(self):

        # --- Step 1: Get waypoints ---
        waypoints_resp = requests.get(f"{self.SERVER}/api/next_waypoints")
        waypoints = waypoints_resp.json().get("waypoints", [])

        # --- Step 2: Get latest event IDs per building ---
        event_ids_resp = requests.get(f"{self.SERVER}/api/latest_events")
        event_map = event_ids_resp.json().get("latest_events", {})

        # --- Step 4: Send back inspection images & mark as completed ---
        for building, image_path in self.IMAGE_MAP.items():
            event_id = event_map.get(building)
            if event_id:
                try:
                    with open(image_path, "rb") as img_file:
                        encoded_image = base64.b64encode(img_file.read()).decode("utf-8")

                    if self.confidence:
                        payload = {
                            "status": "completed",
                            "description": f"Drone inspection completed for Building {building}.",
                            "image": encoded_image
                        }
                    else:
                        payload = {
                            "status": "completed",
                            "description": f"Drone inspection completed for Building {building}.",
                            "vlm_description": "No Crack detected for this waypoint."
                        }

                    complete_resp = requests.put(
                        f"{self.SERVER}/api/building/event/{event_id}",
                        json=payload
                    )

                    if complete_resp.ok:
                        print(f"Uploaded inspection image for Building {building} (event {event_id})")
                    else:
                        print(f"Failed to upload image for {building}: {complete_resp.text}")

                except Exception as e:
                    print(f"Error reading or sending image for {building}: {e}")
        self.IMAGE_MAP = {}

    def call_stop_image_processing_service(self, client, service_name: str):
        # Wait for the service to be available
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} service not available, waiting...')
        self.get_logger().info(f'Calling {service_name} service...')
        req = Trigger.Request()
        future = client.call_async(req)
        # rclpy.spin_until_future_complete(self, future)
        # if future.result() is not None:
        #     response = future.result()
        #     self.get_logger().info(
        #         f'{service_name} response: success={response.success}, message="{response.message}"'
        #     )
        # else:
        #     self.get_logger().error(f'Exception calling {service_name} service: {future.exception()}')

def main(args=None):
    rclpy.init(args=args)
    node = DroneWaypointProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
