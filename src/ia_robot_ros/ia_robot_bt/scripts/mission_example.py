#!/usr/bin/env python3
"""
Example: Simple mission execution using behavior tree

This example shows how to programmatically trigger a mission
and monitor its progress.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import time


class MissionExample(Node):
    """Example client for mission behavior tree"""
    
    def __init__(self):
        super().__init__('mission_example')
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.trigger_pub = self.create_publisher(Bool, '/mission_trigger', 10)
        
        # Subscriber
        self.status_sub = self.create_subscription(
            String,
            '/mission_status',
            self.status_callback,
            10
        )
        
        # Service clients
        self.start_client = self.create_client(Trigger, '/start_mission')
        self.abort_client = self.create_client(Trigger, '/abort_mission')
        
        self.current_status = "UNKNOWN"
        
    def status_callback(self, msg):
        """Track mission status"""
        if msg.data != self.current_status:
            self.get_logger().info(f'Mission status: {msg.data}')
            self.current_status = msg.data
    
    def send_goal_pose(self, x, y):
        """Send a goal pose to the mission coordinator"""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Published goal: ({x:.1f}, {y:.1f})')
    
    def trigger_mission(self):
        """Trigger mission via topic"""
        msg = Bool()
        msg.data = True
        self.trigger_pub.publish(msg)
        self.get_logger().info('Mission triggered via topic')
    
    def start_mission_service(self):
        """Start mission via service call"""
        if not self.start_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Start mission service not available')
            return False
        
        request = Trigger.Request()
        future = self.start_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Service result: {response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False
    
    def abort_mission(self):
        """Abort current mission"""
        if not self.abort_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Abort mission service not available')
            return False
        
        request = Trigger.Request()
        future = self.abort_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Abort result: {response.message}')
            return response.success
        else:
            self.get_logger().error('Abort service call failed')
            return False


def example_1_topic_trigger(node):
    """Example 1: Trigger mission via topic"""
    print("\n" + "="*60)
    print("Example 1: Topic-based mission trigger")
    print("="*60)
    
    # Send goal pose
    node.send_goal_pose(2.0, 1.0)
    time.sleep(1.0)
    
    # Trigger mission
    node.trigger_mission()
    
    print("Mission started via topic. Monitor /mission_status")
    print("="*60 + "\n")


def example_2_service_trigger(node):
    """Example 2: Trigger mission via service"""
    print("\n" + "="*60)
    print("Example 2: Service-based mission trigger")
    print("="*60)
    
    # Send goal pose
    node.send_goal_pose(3.0, 2.0)
    time.sleep(1.0)
    
    # Start via service
    success = node.start_mission_service()
    
    if success:
        print("Mission started via service")
    else:
        print("Failed to start mission")
    
    print("="*60 + "\n")


def example_3_multiple_missions(node):
    """Example 3: Multiple sequential missions"""
    print("\n" + "="*60)
    print("Example 3: Multiple sequential missions")
    print("="*60)
    
    goals = [
        (1.0, 1.0),
        (2.0, 2.0),
        (3.0, 1.0),
    ]
    
    for i, (x, y) in enumerate(goals, 1):
        print(f"\n--- Mission {i}/3 ---")
        node.send_goal_pose(x, y)
        time.sleep(1.0)
        node.start_mission_service()
        
        # Wait for completion (in real scenario, wait for status change)
        print(f"Mission {i} started. Waiting for completion...")
        time.sleep(5.0)  # Simulated wait
    
    print("\nAll missions queued")
    print("="*60 + "\n")


def example_4_abort(node):
    """Example 4: Abort a running mission"""
    print("\n" + "="*60)
    print("Example 4: Abort mission")
    print("="*60)
    
    # Start a mission
    node.send_goal_pose(5.0, 5.0)
    time.sleep(1.0)
    node.start_mission_service()
    
    print("Mission started. Waiting 3 seconds before abort...")
    time.sleep(3.0)
    
    # Abort
    node.abort_mission()
    
    print("Mission aborted")
    print("="*60 + "\n")


def main():
    rclpy.init()
    
    print("\n" + "="*60)
    print("Behavior Tree Mission Examples")
    print("="*60)
    print("\nMake sure the mission_bt_node is running:")
    print("  ros2 run ia_robot_bt mission_bt_node")
    print("\n" + "="*60)
    
    node = MissionExample()
    
    # Give time for connections
    print("\nWaiting for connections...")
    time.sleep(2.0)
    
    try:
        # Run examples
        example_1_topic_trigger(node)
        time.sleep(2.0)
        
        example_2_service_trigger(node)
        time.sleep(2.0)
        
        # Uncomment to run more examples:
        # example_3_multiple_missions(node)
        # example_4_abort(node)
        
        print("\nExamples complete. Spinning to monitor status...")
        print("Press Ctrl+C to exit\n")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
