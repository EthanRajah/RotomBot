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
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/vicon/ROB498_Drone/ROB498_Drone',
            10)

        self.get_logger().info('Dummy Vicon Republisher Node Started')

    def pose_callback(self, msg):
        # Create a new PoseStamped message
        new_msg = PoseStamped()

        # Copy header
        new_msg.header = msg.header

        # Copy pose data
        new_msg.pose = msg.pose

        # Publish to the new topic
        self.pose_pub.publish(new_msg)

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
