#!/usr/bin/env python3

import rclpy
import rclpy.qos
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry  # use Odometry since it has both pose and twist
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix


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

        self.T_2_0 = np.eye(4)

    def realsense_callback(self, msg):
        # Create new PoseStamped message for MAVROS
        # self.get_logger().info('Publishing')

        # Call T_0_2 to get the transform from the camera frame to the map frame
        t, q = msg.pose.pose.position, msg.pose.pose.orientation
        T_3_2 = quaternion_matrix([q.x, q.y, q.z, q.w])    
        T_3_2[:3, 3] = [t.x, t.y, t.z]    

        # Transform we want to publish: T_3_0 = T_2_0 * T_3_2
        T_3_0 = np.dot(self.T_2_0, T_3_2)
        # Get position and quaternion
        p = T_3_0[:3, 3]
        
        # Convert Rotation Matrix to Quaternion (Fix)
        q_matrix = quaternion_from_matrix(T_3_0)

        # Convert Quaternion to Euler Angles
        euler_angles = euler_from_quaternion(q_matrix)

        # Convert back to Quaternion
        q = quaternion_from_euler(*euler_angles)

        mavros_msg = PoseStamped()

        # Copy header
        mavros_msg.header = msg.header
        mavros_msg.header.frame_id = 'map'


        mavros_msg.pose.position.x, mavros_msg.pose.position.y, mavros_msg.pose.position.z = p[0], p[1], p[2]

        # Copy orientation data
        mavros_msg.pose.orientation.x = q[0]
        mavros_msg.pose.orientation.y = q[1]
        mavros_msg.pose.orientation.z = q[2]
        mavros_msg.pose.orientation.w = q[3]

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