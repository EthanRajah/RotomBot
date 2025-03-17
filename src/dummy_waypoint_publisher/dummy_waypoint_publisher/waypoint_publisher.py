#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import time  # For controlling the loop rate

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Create a publisher for waypoints
        self.publisher = self.create_publisher(PoseArray, '/rob498_drone_4/comm/waypoints', 10)

        # Define waypoints manually (x, y, z, qx, qy, qz, qw)
        # self.waypoints = [
        #     [1.0, 1.0, 0.5, 0, 0, 0, 1],  # Waypoint 1
        #     [-1.0, -1.0, 0.5, 0, 0, 0, 1],  # Waypoint 2
        # ]

        # self.waypoints = [
        #     [-2.3, 2.3, 0.5, 0, 0, 0, 1],  # Waypoint 1
        #     [0, 2.3, 0.5, 0, 0, 0, 1],  # Waypoint 2
        #     [2.8, 2.3, 0.5, 0, 0, 0, 1],  # Waypoint 3
        #     [1.5, 0.1, 0.5, 0, 0, 0, 1],  # Waypoint 4
        #     [2.0, -2.3, 0.5, 0, 0, 0, 1],  # Waypoint 5
        #     [1.0, -2.8, 0.5, 0, 0, 0, 1],  # Waypoint 6
        #     [-2.5, -2.5, 0.5, 0, 0, 0, 1],  # Waypoint 7
        # ]

        self.waypoints = [
            [-2.3, 2.3, 1.9, 0, 0, 0, 1],  # Waypoint 1
            [0, 2.3, 1.6, 0, 0, 0, 1],  # Waypoint 2
            [2.8, 2.3, 1.0, 0, 0, 0, 1],  # Waypoint 3
            [1.5, 0.1, 2.0, 0, 0, 0, 1],  # Waypoint 4
            [2.0, -2.3, 1.8, 0, 0, 0, 1],  # Waypoint 5
            [1.0, -2.8, 1.6, 0, 0, 0, 1],  # Waypoint 6
            [-2.5, -2.5, 1.0, 0, 0, 0, 1],  # Waypoint 7
        ]

        self.get_logger().info("Starting continuous waypoint publishing...")

        # Start the publishing loop
        self.publish_waypoints_loop()

    def publish_waypoints_loop(self):
        """ Continuously publishes waypoints at a fixed rate. """
        rate = 2.0  # Publish every 2 seconds
        while rclpy.ok():
            waypoint_array = PoseArray()
            waypoint_array.header.stamp = self.get_clock().now().to_msg()
            waypoint_array.header.frame_id = "vicon/world"  # Set the frame

            for waypoint in self.waypoints:
                pose = Pose()
                pose.position.x = float(waypoint[0])
                pose.position.y = float(waypoint[1])
                pose.position.z = float(waypoint[2])
                pose.orientation.x = float(waypoint[3])
                pose.orientation.y = float(waypoint[4])
                pose.orientation.z = float(waypoint[5])
                pose.orientation.w = float(waypoint[6])
                waypoint_array.poses.append(pose)

            self.publisher.publish(waypoint_array)
            self.get_logger().info(f"Published {len(waypoint_array.poses)} waypoints")

            time.sleep(rate)  # Control the publishing rate

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)  # Keep the node alive
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
