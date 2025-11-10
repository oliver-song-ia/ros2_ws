#!/usr/bin/env python3
"""
Simple script to view full mission status messages
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusViewer(Node):
    def __init__(self):
        super().__init__('status_viewer')
        self.subscription = self.create_subscription(
            String,
            '/mission_status',
            self.status_callback,
            10
        )
        self.get_logger().info('Listening to /mission_status...')
        self.get_logger().info('='*80)
    
    def status_callback(self, msg):
        # Clear screen (optional)
        # print('\033[2J\033[H', end='')
        
        print('\n' + '='*80)
        print('MISSION STATUS:')
        print('='*80)
        print(msg.data)
        print('='*80 + '\n')


def main(args=None):
    rclpy.init(args=args)
    viewer = StatusViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
