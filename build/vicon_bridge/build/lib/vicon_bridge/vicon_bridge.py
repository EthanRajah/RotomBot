#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix


class ViconBridge(Node):
    def __init__(self):
        super().__init__('vicon_bridge')
        self.transform = np.array([0, 0.089, 0.012])

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

        self.publish = False

    def vicon_callback(self, msg):
        # Create new PoseStamped message for MAVROS
        mavros_msg = PoseStamped()

        # Copy header
        mavros_msg.header = msg.header
        mavros_msg.header.frame_id = 'map'


        # Copy vicon position data and transform to cube frame
        # Get rotation matrix from current quaternion
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        R = quaternion_matrix(quat)[:3, :3]
        # Transform the self.transform data so we add it in the cube frame
        self.cur_transform = np.dot(R, self.transform)
        mavros_msg.pose.position = msg.pose.position
        mavros_msg.pose.position.x += self.cur_transform[0]
        mavros_msg.pose.position.y += self.cur_transform[1]
        mavros_msg.pose.position.z += self.cur_transform[2]

        # Copy orientation data
        mavros_msg.pose.orientation = msg.pose.orientation

        # Publish the transformed message
        if self.publish:
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