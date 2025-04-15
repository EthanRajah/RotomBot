#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class DummyViconRepublisher(Node):
    def __init__(self):
        super().__init__('dummy_vicon_republisher')

        # Subscriber to MAVROS pose topic
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            rclpy.qos.qos_profile_system_default)

        # Publisher to Vicon topic
        self.pose_pub_a = self.create_publisher(
            PoseStamped,
            '/vicon/ROB498_Obstacle_A/ROB498_Obstacle_A',
            10)
        self.pose_pub_b = self.create_publisher(
            PoseStamped,
            '/vicon/ROB498_Obstacle_B/ROB498_Obstacle_B',
            10)
        self.pose_pub_c= self.create_publisher(
            PoseStamped,
            '/vicon/ROB498_Obstacle_C/ROB498_Obstacle_C',
            10)
        self.pose_pub_d = self.create_publisher(
            PoseStamped,
            '/vicon/ROB498_Obstacle_D/ROB498_Obstacle_D',
            10)

        self.get_logger().info('Dummy Vicon Republisher Node Started')

    def pose_callback(self, msg):
        # Create a new PoseStamped message
        new_msg = PoseStamped()

        # Copy header
        new_msg.header = msg.header
        new_msg.header.frame_id = "vicon/world"  # Set the frame

        # # Copy pose data
        # new_msg.pose = msg.pose

        # Set pose to 90 degree yaw rotation
        new_msg.pose.position.x = 1.75
        new_msg.pose.position.y = -1.75
        new_msg.pose.position.z = 1.0
        new_msg.pose.orientation.x = 0.0
        new_msg.pose.orientation.y = 0.0
        new_msg.pose.orientation.z = -0.707
        new_msg.pose.orientation.w = 0.707

        # Publish to the new topic: publish different waypoints in x and y (all we care about)
        self.pose_pub_a.publish(new_msg)

        new_msg.pose.position.x = -1.75
        new_msg.pose.position.y = -1.75
        self.pose_pub_b.publish(new_msg)

        new_msg.pose.position.x = 1.75
        new_msg.pose.position.y = 1.75
        self.pose_pub_c.publish(new_msg)

        new_msg.pose.position.x = -1.75
        new_msg.pose.position.y = 1.75
        self.pose_pub_d.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyViconRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()