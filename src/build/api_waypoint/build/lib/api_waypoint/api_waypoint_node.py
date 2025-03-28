#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import requests
import time
from threading import Lock
from geometry_msgs.msg import PoseArray, Pose, PoseStamped

class WaypointProcessorNode(Node):
    """
    ROS 2 node that periodically checks for published waypoints and updates their status to enroute.
    """
    
    def __init__(self):
        super().__init__('waypoint_processor_node')
        
        # Parameters
        # self.ip = "http://10.42.0.103:5072"
        self.ip = "http://100.66.221.84:5072"
        self.declare_parameter('check_interval_sec', 5.0)  # How often to check for waypoints
        self.declare_parameter('api_base_url', self.ip)
        
        # Get parameters
        self.check_interval = self.get_parameter('check_interval_sec').value
        self.api_base_url = self.get_parameter('api_base_url').value
        
        # Create timer for periodic waypoint checking
        self.timer = self.create_timer(self.check_interval, self.check_and_update_waypoints)

        # Lookup table for buildings -> position + quaternion, make sure all floats
        # Hard code z and orientation so that it matches where the crack is on the pillar (phase)
        self.lookup = {"A": [0.0, 0.55, 1.0, 0.0, 0.0, 0.707, 0.707], # 90 degree yaw
                       "B": [-0.55, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0], # 180 yaw
                       "C": [0.0, -0.55, 1.0, 0.0, 0.0, 0.707, -0.707], # 270 yaw
                       "D": [0.55, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]} # 0 yaw
        
        # For thread safety when accessing shared resources
        self.lock = Lock()
        
        self.get_logger().info('Waypoint Processor Node has started')

        # Create a publisher for waypoints
        self.publisher = self.create_publisher(PoseArray, '/rob498_drone_4/comm/waypoints', 10)

        # Create array of site ids to publish to the waypoint topic
        self.loc_keys = []

        # Get obstacle positions (one subscriber for each A, B, C, D)
        self.obs_a = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Obstacle_A/ROB498_Obstacle_A',
            self.vicon_callback_a,
            10)
        self.obs_b = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Obstacle_B/ROB498_Obstacle_B',
            self.vicon_callback_b,
            10)
        self.obs_c = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Obstacle_C/ROB498_Obstacle_C',
            self.vicon_callback_c,
            10)
        self.obs_d = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Obstacle_D/ROB498_Obstacle_D',
            self.vicon_callback_d,
            10)

    def check_and_update_waypoints(self):
        """
        Periodic callback to check for new waypoints and update their status.
        """
        self.get_logger().info('Checking for waypoints...')
        self.get_logger().info(f'Lookup table: {self.lookup}')
        try:
            with self.lock:
                # Step 1: Get waypoints
                waypoints_url = f"{self.api_base_url}/api/next_waypoints"
                self.get_logger().debug(f"Getting waypoints from: {waypoints_url}")
                waypoints_resp = requests.get(waypoints_url, timeout=5.0)
                
                if not waypoints_resp.ok:
                    self.get_logger().error(f"Failed to get waypoints: {waypoints_resp.status_code} - {waypoints_resp.text}")
                    return
                
                waypoints = waypoints_resp.json().get("waypoints", [])
                
                if not waypoints:
                    self.get_logger().info("No waypoints received")
                    self.publish_poses()
                    return
                
                self.get_logger().info(f"Received {len(waypoints)} waypoints: {waypoints}")
                
                # Step 2: Get latest event IDs per building
                events_url = f"{self.api_base_url}/api/latest_events"
                self.get_logger().debug(f"Getting event IDs from: {events_url}")
                event_ids_resp = requests.get(events_url, timeout=5.0)
                
                if not event_ids_resp.ok:
                    self.get_logger().error(f"Failed to get event IDs: {event_ids_resp.status_code} - {event_ids_resp.text}")
                    return
                
                event_map = event_ids_resp.json().get("latest_events", {})
                
                # Step 3: Update each building with status "sent" to "en_route", and publish the pose of each waypoint using lookup table
                for building in waypoints:
                    event_id = event_map.get(building)
                    if event_id:
                        # Get the current status of the event
                        event_url = f"{self.api_base_url}/api/event/{event_id}"
                        event_resp = requests.get(event_url, timeout=5.0)
                        
                        if not event_resp.ok:
                            self.get_logger().error(f"Failed to get event for building {building}: {event_resp.status_code} - {event_resp.text}")
                            continue
                            
                        event_data = event_resp.json().get("event", {})
                        current_status = event_data.get("status")
                        
                        # Only update if status is "sent"
                        if current_status == "sent":
                            self.get_logger().info(f"Marking building {building} (event {event_id}) as en_route")
                            update_url = f"{self.api_base_url}/api/building/event/{event_id}"
                            update_resp = requests.put(
                                update_url,
                                json={"status": "en_route"},
                                timeout=5.0
                            )
                            
                            if update_resp.ok:
                                self.get_logger().info(f"Building {building} marked as en_route")
                            else:
                                self.get_logger().error(f"Failed to update {building}: {update_resp.status_code} - {update_resp.text}")
                        else:
                            self.get_logger().debug(f"Skipping building {building} with status '{current_status}'")
                    else:
                        self.get_logger().warn(f"No event ID found for building {building}")
                    # Append building id to self.loc_keys
                    self.loc_keys.append(building)
                    self.publish_poses()
                        
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Request error during waypoint processing: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Error processing waypoints: {str(e)}")

    def publish_poses(self):
        """
        Publishes self.loc_keys to the waypoint topic for use in comm node.
        """
        # Only publish if you have buildings
        if self.loc_keys == []:
            return

        # initialize pose array
        waypoint_array = PoseArray()
        waypoint_array.header.stamp = self.get_clock().now().to_msg()
        waypoint_array.header.frame_id = "vicon/world"  # Set the frame

        # Loop over buildings, call lookup to get position + quaternion
        for building in self.loc_keys:
            coords = self.lookup[building]

            # Publish
            pose = Pose()
            pose.position.x = float(coords[0])
            pose.position.y = float(coords[1])
            pose.position.z = float(coords[2])
            pose.orientation.x = float(coords[3])
            pose.orientation.y = float(coords[4])
            pose.orientation.z = float(coords[5])
            pose.orientation.w = float(coords[6])
            waypoint_array.poses.append(pose)

        self.publisher.publish(waypoint_array)

    def vicon_callback_a(self, msg):
        # Create new PoseStamped message for MAVROS
        mavros_msg = PoseStamped()

        # Copy header
        mavros_msg.header = msg.header
        mavros_msg.header.frame_id = 'map'

        # Copy vicon position data 
        mavros_msg.pose.position = msg.pose.position

        # Copy orientation data:
        mavros_msg.pose.orientation = msg.pose.orientation

        # Update lookup at this key
        coords = [msg.pose.position.x + self.lookup['A'][0],
                  msg.pose.position.y + self.lookup['A'][1]]
        self.lookup['A'][:2] = coords

    def vicon_callback_b(self, msg):
        # Create new PoseStamped message for MAVROS
        mavros_msg = PoseStamped()

        # Copy header
        mavros_msg.header = msg.header
        mavros_msg.header.frame_id = 'map'

        # Copy vicon position data 
        mavros_msg.pose.position = msg.pose.position

        # Copy orientation data
        mavros_msg.pose.orientation = msg.pose.orientation

        # Update lookup at this key
        coords = [msg.pose.position.x + self.lookup['B'][0],
                  msg.pose.position.y + self.lookup['B'][1]]
        self.lookup['B'][:2] = coords

    def vicon_callback_c(self, msg):
        # Create new PoseStamped message for MAVROS
        mavros_msg = PoseStamped()

        # Copy header
        mavros_msg.header = msg.header
        mavros_msg.header.frame_id = 'map'

        # Copy vicon position data 
        mavros_msg.pose.position = msg.pose.position

        # Copy orientation data
        mavros_msg.pose.orientation = msg.pose.orientation

        # Update lookup at this key
        coords = [msg.pose.position.x + self.lookup['C'][0],
                  msg.pose.position.y + self.lookup['C'][1]]
        self.lookup['C'][:2] = coords
    
    def vicon_callback_d(self, msg):
        # Create new PoseStamped message for MAVROS
        mavros_msg = PoseStamped()

        # Copy header
        mavros_msg.header = msg.header
        mavros_msg.header.frame_id = 'map'

        # Copy vicon position data 
        mavros_msg.pose.position = msg.pose.position

        # Copy orientation data
        mavros_msg.pose.orientation = msg.pose.orientation

        # Update lookup at this key
        coords = [msg.pose.position.x + self.lookup['D'][0],
                  msg.pose.position.y + self.lookup['D'][1]]
        self.lookup['D'][:2] = coords
        
def main(args=None):
    rclpy.init(args=args)
    node = WaypointProcessorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by keyboard interrupt')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
