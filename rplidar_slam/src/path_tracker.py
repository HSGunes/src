#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import math

class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')
        
        # Publisher for the path
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Path message
        self.path = Path()
        self.path.header.frame_id = "map"
        
        # Last position for distance check
        self.last_x = None
        self.last_y = None
        self.min_distance = 0.1  # Minimum distance to add new point (10cm)
        
        # Timer to periodically check robot position
        self.timer = self.create_timer(0.1, self.update_path)  # 10Hz
        
        self.get_logger().info('Path tracker node started')

    def update_path(self):
        try:
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # Check if we moved enough to add a new point
            if self.last_x is None or self.distance_moved(x, y) > self.min_distance:
                # Create new pose
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation = transform.transform.rotation
                
                # Add to path
                self.path.poses.append(pose)
                self.path.header.stamp = pose.header.stamp
                
                # Update last position
                self.last_x = x
                self.last_y = y
                
                # Limit path length to prevent memory issues
                if len(self.path.poses) > 10000:
                    self.path.poses = self.path.poses[-5000:]
                
                # Publish path
                self.path_publisher.publish(self.path)
                
        except TransformException as ex:
            # Transform not available yet, just skip
            pass

    def distance_moved(self, x, y):
        if self.last_x is None:
            return float('inf')
        return math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)

def main(args=None):
    rclpy.init(args=args)
    path_tracker = PathTracker()
    
    try:
        rclpy.spin(path_tracker)
    except KeyboardInterrupt:
        pass
    
    path_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 