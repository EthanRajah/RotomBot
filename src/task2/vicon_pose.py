#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class ViconBridge(Node):
    def __init__(self):
        super().__init__('vicon_bridge')

        # Create subscriber for Vicon pose data
        self.vicon_sub = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Drone/ROB498_Drone',
            self.vicon_callback,
            10)

        # Create publisher for MAVROS vision pose
        self.mavros_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10)

        self.get_logger().info('Vicon Bridge Node Started')

    def vicon_callback(self, msg):
        # Create new PoseStamped message for MAVROS
        mavros_msg = PoseStamped()

        # Copy header
        mavros_msg.header = msg.header

        # Copy position data
        mavros_msg.pose.position = msg.pose.position

        # Copy orientation data
        mavros_msg.pose.orientation = msg.pose.orientation

        # Publish the transformed message
        self.mavros_pub.publish(mavros_msg)


def main(args=None):
    rclpy.init(args=args)

    vicon_bridge = ViconBridge()

    try:
        rclpy.spin(vicon_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        vicon_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()