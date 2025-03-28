#!/usr/bin/env python3
import rclpy
import rclpy.qos
import numpy as np
import csv
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import os


class obs_grabber(Node):
    def __init__(self, combined_file):
        super().__init__('obs_grabber')

        # Store CSV file handle
        self.combined_file = combined_file
        
        # Create CSV writer
        self.combined_writer = csv.writer(combined_file)
        
        # Write CSV header
        # self.combined_writer.writerow([
        #     'timestamp', 
        #     'realsense_pos_x', 'realsense_pos_y', 'realsense_pos_z', 
        #     'realsense_orient_x', 'realsense_orient_y', 'realsense_orient_z', 'realsense_orient_w',
        #     'vicon_pos_x', 'vicon_pos_y', 'vicon_pos_z', 
        #     'vicon_orient_x', 'vicon_orient_y', 'vicon_orient_z', 'vicon_orient_w'
        # ])
        self.combined_writer.writerow([
            'timestamp', 
            'vicon_pos_x', 'vicon_pos_y', 'vicon_pos_z', 
            'vicon_orient_x', 'vicon_orient_y', 'vicon_orient_z', 'vicon_orient_w'
        ])

        # Create subscribers using message_filters
        # self.realsense_sub = Subscriber(self, Odometry, '/camera/pose/sample', 
        #     qos_profile=rclpy.qos.QoSProfile(
        #         reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        #         durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        #         depth=10))
        
        self.vicon_sub = Subscriber(self, PoseStamped, '/vicon/ROB498_Drone/ROB498_Drone', 10)
        
        # Create synchronizer for the two topics with a time tolerance of 0.1 seconds
        self.ts = ApproximateTimeSynchronizer(
            [self.vicon_sub], 
            queue_size=10, 
            slop=0.01)
        
        # Register callback
        self.ts.registerCallback(self.synchronized_callback)
        
        self.get_logger().info('Subscribed to both topics with time synchronization')

    def synchronized_callback(self, vicon_msg):
        # Get the synchronized timestamp (average of both)
        timestamp = vicon_msg.header.stamp.sec + vicon_msg.header.stamp.nanosec * 1e-9
        
        self.get_logger().info('Recorded synchronized data')
        
        # Write both data points to CSV
        self.combined_writer.writerow([
            timestamp,
            # Vicon data
            vicon_msg.pose.position.x,
            vicon_msg.pose.position.y,
            vicon_msg.pose.position.z,
            vicon_msg.pose.orientation.x,
            vicon_msg.pose.orientation.y,
            vicon_msg.pose.orientation.z,
            vicon_msg.pose.orientation.w
        ])


def main(args=None):
    rclpy.init(args=args)

    # Open CSV file
    output_dir = 'src/exer3_control/exer3_control'
    os.makedirs(output_dir, exist_ok=True)
    combined_data_file = open(os.path.join(output_dir, 'vicon_data_obs.csv'), 'w', newline='')

    # Create grabber instance with CSV file handle
    grabber_class = obs_grabber(combined_data_file)

    try:
        rclpy.spin(grabber_class)
    except KeyboardInterrupt:
        pass
    finally:
        # Close CSV file before shutting down
        combined_data_file.close()
        grabber_class.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()