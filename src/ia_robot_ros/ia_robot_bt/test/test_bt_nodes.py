#!/usr/bin/env python3
"""
Simple test script for behavior tree implementation
"""

import sys
import unittest
from unittest.mock import MagicMock, patch
import py_trees

# Mock ROS to allow testing without full ROS environment
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.action'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_srvs'] = MagicMock()
sys.modules['nav2_msgs'] = MagicMock()
sys.modules['action_msgs'] = MagicMock()

from ia_robot_bt.bt_conditions import IsHumanPoseAvailable, IsMissionTriggered


class TestBehaviorTreeConditions(unittest.TestCase):
    """Test behavior tree condition nodes"""
    
    def setUp(self):
        """Setup test blackboard"""
        self.blackboard = py_trees.blackboard.Client(name="TestClient")
        self.blackboard.register_key(key='human_pose', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='mission_triggered', access=py_trees.common.Access.WRITE)
    
    def test_human_pose_available_success(self):
        """Test IsHumanPoseAvailable returns SUCCESS when pose is available"""
        # Setup
        self.blackboard.set('human_pose', MagicMock())
        condition = IsHumanPoseAvailable("Test", self.blackboard)
        
        # Execute
        status = condition.update()
        
        # Verify
        self.assertEqual(status, py_trees.common.Status.SUCCESS)
    
    def test_human_pose_available_failure(self):
        """Test IsHumanPoseAvailable returns FAILURE when pose is None"""
        # Setup
        self.blackboard.set('human_pose', None)
        condition = IsHumanPoseAvailable("Test", self.blackboard)
        
        # Execute
        status = condition.update()
        
        # Verify
        self.assertEqual(status, py_trees.common.Status.FAILURE)
    
    def test_mission_triggered_success(self):
        """Test IsMissionTriggered returns SUCCESS when triggered"""
        # Setup
        self.blackboard.set('mission_triggered', True)
        condition = IsMissionTriggered("Test", self.blackboard)
        
        # Execute
        status = condition.update()
        
        # Verify
        self.assertEqual(status, py_trees.common.Status.SUCCESS)
    
    def test_mission_triggered_failure(self):
        """Test IsMissionTriggered returns FAILURE when not triggered"""
        # Setup
        self.blackboard.set('mission_triggered', False)
        condition = IsMissionTriggered("Test", self.blackboard)
        
        # Execute
        status = condition.update()
        
        # Verify
        self.assertEqual(status, py_trees.common.Status.FAILURE)


class TestBehaviorTreeStructure(unittest.TestCase):
    """Test behavior tree structure and composition"""
    
    def test_sequence_success(self):
        """Test Sequence returns SUCCESS when all children succeed"""
        # Create mock behaviors that succeed
        child1 = MagicMock()
        child1.update = MagicMock(return_value=py_trees.common.Status.SUCCESS)
        child1.status = py_trees.common.Status.SUCCESS
        
        child2 = MagicMock()
        child2.update = MagicMock(return_value=py_trees.common.Status.SUCCESS)
        child2.status = py_trees.common.Status.SUCCESS
        
        # Create sequence
        sequence = py_trees.composites.Sequence(
            name="Test Sequence",
            memory=False,
            children=[child1, child2]
        )
        
        # Setup and tick
        sequence.setup_with_descendants()
        sequence.tick_once()
        
        # Verify
        self.assertEqual(sequence.status, py_trees.common.Status.SUCCESS)
    
    def test_sequence_failure(self):
        """Test Sequence returns FAILURE when a child fails"""
        # Create mock behaviors
        child1 = MagicMock()
        child1.update = MagicMock(return_value=py_trees.common.Status.SUCCESS)
        child1.status = py_trees.common.Status.SUCCESS
        
        child2 = MagicMock()
        child2.update = MagicMock(return_value=py_trees.common.Status.FAILURE)
        child2.status = py_trees.common.Status.FAILURE
        
        # Create sequence
        sequence = py_trees.composites.Sequence(
            name="Test Sequence",
            memory=False,
            children=[child1, child2]
        )
        
        # Setup and tick
        sequence.setup_with_descendants()
        sequence.tick_once()
        
        # Verify sequence fails when child fails
        self.assertEqual(sequence.status, py_trees.common.Status.FAILURE)


if __name__ == '__main__':
    unittest.main()
