#!/usr/bin/env python3
"""
Behavior Tree Visualization Tool - Simplified

This script creates a visual representation of the simplified mission behavior tree
without requiring ROS to be running.
"""

import py_trees
from py_trees.composites import Sequence


def create_mock_tree():
    """
    Create a mock version of the simplified mission behavior tree for visualization
    """
    
    # Mock behavior nodes
    wait_for_goal = py_trees.behaviours.Success(name="Wait for Goal Pose")
    wait_nav_complete = py_trees.behaviours.Success(name="Wait for Navigation Complete")
    start_manipulation = py_trees.behaviours.Success(name="Start Manipulation")
    wait_complete = py_trees.behaviours.Success(name="Wait Manipulation Complete")
    stop_manipulation = py_trees.behaviours.Success(name="Stop Manipulation")
    
    # Manipulation sequence
    manipulation_sequence = Sequence(
        name="Manipulation Phase",
        memory=False,
        children=[
            start_manipulation,
            wait_complete,
            stop_manipulation,
        ]
    )
    
    # Main mission sequence
    mission_sequence = Sequence(
        name="Mission Sequence",
        memory=False,
        children=[
            wait_for_goal,
            wait_nav_complete,
            manipulation_sequence,
        ]
    )
    
    return py_trees.trees.BehaviourTree(mission_sequence)
    
    # Main mission sequence
    mission_sequence = Sequence(
        name="Mission Sequence",
        memory=False,
        children=[
            wait_for_goal,
            wait_nav_complete,
            manipulation_sequence,
        ]
    )
    
    return py_trees.trees.BehaviourTree(mission_sequence)


def main():
    """Visualize the behavior tree"""
    
    print("=" * 80)
    print("Mission Coordinator Behavior Tree Visualization (Simplified)")
    print("=" * 80)
    print()
    
    # Create the tree
    tree = create_mock_tree()
    
    # Print ASCII representation
    print("Tree Structure:")
    print("-" * 80)
    print(py_trees.display.ascii_tree(tree.root))
    print()
    
    # Print detailed information
    print("Tree Details:")
    print("-" * 80)
    print(f"Total nodes: {len(list(tree.root.iterate()))}")
    print(f"Max depth: {_get_tree_depth(tree.root)}")
    print()
    
    print("Node Types:")
    print("-" * 80)
    for node in tree.root.iterate():
        node_type = type(node).__name__
        print(f"  {node.name:45} [{node_type}]")
    print()
    
    print("Execution Flow:")
    print("-" * 80)
    print("1. Wait for goal pose on /goal_pose topic")
    print("2. Goal received → Nav2 starts navigation automatically")
    print("3. Monitor Nav2 status topic for completion")
    print("4. On navigation SUCCESS:")
    print("   - Start manipulation")
    print("     • Launch IK controller")
    print("     • Launch mocap publisher")
    print("5. Wait for manipulation complete")
    print("   - Check process status")
    print("   - Or timeout after 60 seconds")
    print("6. Stop manipulation")
    print("   - Terminate processes")
    print("   - Cleanup")
    print("7. Loop back to step 1 (tree resets automatically)")
    print()
    
    print("Key Simplifications:")
    print("-" * 80)
    print("✓ Nav2 handles navigation (we just monitor)")
    print("✓ No manual navigation action client")
    print("✓ No retry/timeout decorators (Nav2 handles it)")
    print("✓ No trigger mechanism (/goal_pose IS the trigger)")
    print("✓ No services needed")
    print()
    
    print("=" * 80)
    print("To see this tree in action, run:")
    print("  ros2 run ia_robot_bt mission_bt_node")
    print()
    print("Then send a goal:")
    print('  ros2 topic pub /goal_pose geometry_msgs/PoseStamped "...' + '"')
    print("=" * 80)


def _get_tree_depth(node, current_depth=0):
    """Recursively calculate tree depth"""
    if not hasattr(node, 'children') or not node.children:
        return current_depth
    
    max_child_depth = current_depth
    for child in node.children:
        child_depth = _get_tree_depth(child, current_depth + 1)
        max_child_depth = max(max_child_depth, child_depth)
    
    return max_child_depth


if __name__ == '__main__':
    main()
