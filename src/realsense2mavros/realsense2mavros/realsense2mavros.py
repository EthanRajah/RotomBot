#!/usr/bin/env python3

import rclpy
import rclpy.qos
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry  # use Odometry since it has both pose and twist
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class realsense2mavros(Node):
    def __init__(self):
        super().__init__('realsense2mavros')
        # Old self.transform pre crash (2/28)
        # self.transform = np.array([-0.124, 0, 0.1105])
        # Post 2/28
        self.transform = np.array([-0.124, 0, 0.103])

        # Create subscriber for realsense pose data
        self.realsense_sub = self.create_subscription(
            Odometry,
            '/camera/pose/sample',
            self.realsense_callback,
            rclpy.qos.qos_profile_system_default)

        # Create publisher for MAVROS vision pose
        self.mavros_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10)

        self.get_logger().info('Realsense to Mavros Bridge')

        # Only publish if self.publish flag is true
        self.publish = False

    def realsense_callback(self, msg):
        # Create new PoseStamped message for MAVROS
        # self.get_logger().info('Publishing')

        mavros_msg = PoseStamped()

        # Copy header
        mavros_msg.header = msg.header
        mavros_msg.header.frame_id = 'map'

        # Copy position data and transform to cube frame
        mavros_msg.pose.position = msg.pose.pose.position
        mavros_msg.pose.position.x += self.transform[0]
        mavros_msg.pose.position.y += self.transform[1]
        mavros_msg.pose.position.z += self.transform[2]

        # Copy orientation data
        mavros_msg.pose.orientation = msg.pose.pose.orientation

        # Publish the transformed message
        # self.get_logger().info('Realsense Publishing message')
        if self.publish:
            self.mavros_pub.publish(mavros_msg)
        # self.get_logger().info('Publishing complete')


def main(args=None):
    rclpy.init(args=args)

    realsense_bridge = realsense2mavros()

    try:
        rclpy.spin(realsense_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        realsense_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()