#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
import json
import yaml
import math

class AlignmentPointsPublisher(Node):
    def __init__(self):
        super().__init__('alignment_points_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(
            MarkerArray, 
            '/alignment_points_debug', 
            10
        )
        
        # Timer to publish every second
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        # Load alignment points
        self.alignment_points = []
        self.load_alignment_points()
        
        self.get_logger().info(f'Loaded {len(self.alignment_points)} alignment points')

    def load_alignment_points(self):
        # Paths to map files
        json_file = './src/ia_robot_sim/maps/map_0722.json'
        yaml_file = './src/ia_robot_sim/maps/map_0722.yaml'
        
        # Load map parameters from YAML
        map_resolution = 0.05
        origin_x = -11.7
        origin_y = -1.87
        map_height = 175  # Height in pixels for Y-axis flip
        
        try:
            with open(yaml_file, 'r') as f:
                map_data = yaml.safe_load(f)
                map_resolution = map_data['resolution']
                origin_x = map_data['origin'][0]
                origin_y = map_data['origin'][1]
                
            self.get_logger().info(f'Map parameters: resolution={map_resolution}, origin=({origin_x}, {origin_y}), height={map_height}px')
        except Exception as e:
            self.get_logger().warn(f'Could not load map YAML: {e}, using defaults')
        
        # Load alignment points from JSON
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
                
            if 'confirmed_alignment_points' in data:
                for i, point in enumerate(data['confirmed_alignment_points']):
                    # Get pixel coordinates
                    map_x = float(point['mapX'])
                    map_y = float(point['mapY'])
                    json_real_x = float(point['realX'])
                    json_real_y = float(point['realY'])
                    orientation = float(point['orientation'])
                    
                    # Convert to ROS world coordinates with Y-axis flip
                    # Flip Y because image coordinates have origin at top-left, ROS at bottom-left
                    flipped_map_y = map_height - map_y
                    ros_x = float((map_x * map_resolution) + origin_x)
                    ros_y = float((flipped_map_y * map_resolution) + origin_y)
                    
                    alignment_point = {
                        'id': i,
                        'map_x': map_x,
                        'map_y': map_y,
                        'json_real_x': json_real_x,
                        'json_real_y': json_real_y,
                        'ros_x': ros_x,
                        'ros_y': ros_y,
                        'orientation': orientation
                    }
                    
                    self.alignment_points.append(alignment_point)
                    
                    self.get_logger().info(
                        f'Point {i}: mapPix({map_x:.1f},{map_y:.1f}) -> '
                        f'flipped({map_x:.1f},{flipped_map_y:.1f}) -> '
                        f'ROS({ros_x:.3f},{ros_y:.3f}) | '
                        f'JSON_real({json_real_x:.3f},{json_real_y:.3f})'
                    )
                    
        except Exception as e:
            self.get_logger().error(f'Could not load alignment points: {e}')

    def create_marker(self, point, use_ros_coords=True):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "alignment_points"
        marker.id = point['id']
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position - choose coordinate system
        if use_ros_coords:
            marker.pose.position.x = float(point['ros_x'])
            marker.pose.position.y = float(point['ros_y'])
        else:
            marker.pose.position.x = float(point['json_real_x'])
            marker.pose.position.y = float(point['json_real_y'])
            
        marker.pose.position.z = 0.1
        
        # Orientation
        yaw = math.radians(point['orientation'])
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Scale
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # Color - Green for ROS coords, Red for JSON coords
        marker.color.a = 1.0
        if use_ros_coords:
            marker.color.r = 0.0  # Green
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0
            marker.color.b = 0.0
        
        # Lifetime
        marker.lifetime.sec = 2
        
        return marker

    def publish_markers(self):
        if not self.alignment_points:
            return
            
        marker_array = MarkerArray()
        
        # Publish both coordinate systems for comparison
        for point in self.alignment_points:
            # Green markers for ROS coordinates
            ros_marker = self.create_marker(point, use_ros_coords=True)
            marker_array.markers.append(ros_marker)
            
            # Red markers for JSON coordinates (offset ID to avoid conflict)
            json_marker = self.create_marker(point, use_ros_coords=False)
            json_marker.id = point['id'] + 100  # Offset ID
            json_marker.ns = "json_coords"
            marker_array.markers.append(json_marker)
        
        self.publisher.publish(marker_array)
        
        # Only log first time
        if hasattr(self, 'first_publish') and self.first_publish:
            self.get_logger().info(
                f'Published {len(marker_array.markers)} markers: '
                f'Green=ROS coords, Red=JSON coords'
            )
            self.first_publish = False

def main(args=None):
    rclpy.init(args=args)
    
    publisher = AlignmentPointsPublisher()
    publisher.first_publish = True
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()