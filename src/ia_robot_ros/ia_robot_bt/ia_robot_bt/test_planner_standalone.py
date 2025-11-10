#!/usr/bin/env python3
"""
Standalone Test for Planner Behavior Tree
Tests the planner subtree (rest/approach/assist/retreat modes) independently.

Usage:
    python test_planner_standalone.py

Controls:
    - Send mode commands via: ros2 topic pub /set_mode std_msgs/msg/String "data: 'approach'" --once
    - Valid modes: rest, approach, assist, retreat
    - Monitor /target_eef_poses and /mode topics
    - Press Ctrl+C to stop

Requirements:
    - /pose_detection topic publishing (human skeleton detection)
    - /current_poses topic publishing (current robot EEF poses)

### Behavior Tree Visualization
The tree shows the current state with symbols:
- `[✓]` - SUCCESS
- `[✗]` - FAILURE  
- `[*]` - RUNNING
- `[-]` - INVALID
"""

import rclpy
from rclpy.node import Node
import py_trees
from py_trees.trees import BehaviourTree

from planner_bt import create_planner_subtree


class PlannerTestNode(Node):
    """
    Minimal test node to run the planner subtree standalone
    """
    
    def __init__(self):
        super().__init__('planner_test_node')
        
        # Parameters
        self.declare_parameter('tick_rate', 10.0)  # Hz
        self.declare_parameter('print_tree', True)  # Print tree each tick
        self.declare_parameter('print_blackboard', True)  # Print blackboard each tick
        
        self.tick_rate = self.get_parameter('tick_rate').value
        self.print_tree = self.get_parameter('print_tree').value
        self.print_blackboard = self.get_parameter('print_blackboard').value
        
        # Create blackboard
        self.blackboard = py_trees.blackboard.Client(name="PlannerTest")
        
        # Create planner subtree
        self.get_logger().info('Creating planner behavior tree...')
        planner_subtree = create_planner_subtree(self, self.blackboard)
        
        # Create tree
        self.tree = BehaviourTree(planner_subtree)
        
        # Setup tree
        try:
            self.tree.setup(timeout=15)
            self.get_logger().info('Planner behavior tree setup complete')
        except Exception as e:
            self.get_logger().error(f'Failed to setup behavior tree: {e}')
            raise
        
        # Create timer to tick the tree
        timer_period = 1.0 / self.tick_rate
        self.tree_timer = self.create_timer(timer_period, self.tick_tree)
        
        self.get_logger().info('='*70)
        self.get_logger().info('Planner Behavior Tree Standalone Test Started')
        self.get_logger().info('='*70)
        self.get_logger().info('Topics:')
        self.get_logger().info('  Subscribing to:')
        self.get_logger().info('    - /pose_detection (MarkerArray) - human skeleton')
        self.get_logger().info('    - /current_poses (PoseArray) - current robot EEF poses')
        self.get_logger().info('    - /set_mode (String) - mode commands')
        self.get_logger().info('  Publishing to:')
        self.get_logger().info('    - /target_eef_poses (PoseArray) - target poses')
        self.get_logger().info('    - /mode (String) - current mode')
        self.get_logger().info('')
        self.get_logger().info('Mode Commands:')
        self.get_logger().info('  ros2 topic pub /set_mode std_msgs/msg/String "data: \'rest\'" --once')
        self.get_logger().info('  ros2 topic pub /set_mode std_msgs/msg/String "data: \'approach\'" --once')
        self.get_logger().info('  ros2 topic pub /set_mode std_msgs/msg/String "data: \'assist\'" --once')
        self.get_logger().info('  ros2 topic pub /set_mode std_msgs/msg/String "data: \'retreat\'" --once')
        self.get_logger().info('')
        self.get_logger().info('Monitoring:')
        self.get_logger().info('  ros2 topic echo /target_eef_poses')
        self.get_logger().info('  ros2 topic echo /mode')
        self.get_logger().info('='*70)
        self.get_logger().info('Press Ctrl+C to stop...')
        self.get_logger().info('')
    
    def tick_tree(self):
        """Tick the behavior tree"""
        try:
            self.tree.tick()
            
            # Print tree visualization
            if self.print_tree:
                print("\n" + "="*70)
                print("BEHAVIOR TREE STATE:")
                print("="*70)
                print(py_trees.display.unicode_tree(
                    root=self.tree.root,
                    show_status=True
                ))
            
            # Print blackboard state
            if self.print_blackboard:
                print("\n" + "="*70)
                print("BLACKBOARD STATE:")
                print("="*70)
                print(f"  planner_mode: {self.blackboard.get('planner_mode')}")
                print(f"  planner_mode_requested: {self.blackboard.get('planner_mode_requested')}")
                
                joints = self.blackboard.get('planner_joints')
                print(f"  planner_joints: {'Available' if joints is not None else 'None'}")
                
                human_pos = self.blackboard.get('planner_human_pos')
                if human_pos is not None:
                    print(f"  planner_human_pos: [{human_pos[0]:.2f}, {human_pos[1]:.2f}]")
                else:
                    print(f"  planner_human_pos: None")
                
                human_ori = self.blackboard.get('planner_human_ori')
                if human_ori is not None:
                    print(f"  planner_human_ori: {human_ori:.2f} rad ({human_ori*180/3.14159:.1f}°)")
                else:
                    print(f"  planner_human_ori: None")
                
                current_poses = self.blackboard.get('planner_current_poses')
                print(f"  planner_current_poses: {'Available' if current_poses is not None else 'None'}")
                
                rest_target = self.blackboard.get('planner_rest_target')
                print(f"  planner_rest_target: {'Set' if rest_target is not None else 'None'}")
                
                approach_target = self.blackboard.get('planner_approach_target')
                print(f"  planner_approach_target: {'Set' if approach_target is not None else 'None'}")
                
                print("="*70)
                print("")
            
        except Exception as e:
            self.get_logger().error(f'Error ticking tree: {e}')
            import traceback
            traceback.print_exc()
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down planner test node...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # Initialize py_trees logging
    py_trees.logging.level = py_trees.logging.Level.INFO
    
    test_node = PlannerTestNode()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
