#!/usr/bin/env python3
"""
Behavior Tree Condition Nodes - Simplified
"""

import py_trees
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Image


class IsGoalPoseReceived(py_trees.behaviour.Behaviour):
    """
    Check if goal pose has been received
    """
    
    def __init__(self, name, blackboard):
        super().__init__(name)
        self.blackboard = blackboard
    
    def initialise(self):
        """Called when behavior is first ticked or after being reset"""
        # Note: we don't have access to node logger here, so we can't log
        pass
    
    def update(self):
        """Check condition"""
        goal_received = self.blackboard.get('goal_received')
        
        if goal_received:
            self.feedback_message = "Goal pose received"
            # Reset the flag so we wait for next goal
            self.blackboard.set('goal_received', False)
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "Waiting for goal pose"
            return py_trees.common.Status.RUNNING


class IsNavigationComplete(py_trees.behaviour.Behaviour):
    """
    Check if Nav2 navigation is complete by monitoring action status
    """
    
    def __init__(self, name, node, blackboard):
        super().__init__(name)
        self.node = node
        self.blackboard = blackboard
        self.last_logged_status = None
        self.update_count = 0
    
    def initialise(self):
        """Called when behavior is first ticked or after being reset"""
        self.node.get_logger().info('[BT] IsNavigationComplete: initialise() called')
        self.last_logged_status = None
        self.update_count = 0
    
    def update(self):
        """Check navigation status"""
        self.update_count += 1
        nav_status = self.blackboard.get('nav_status')
        
        # Only log if status changed to reduce spam
        if nav_status != self.last_logged_status:
            self.node.get_logger().info(
                f"[BT] IsNavigationComplete.update() #{self.update_count}: "
                f"nav_status={nav_status}, target={GoalStatus.STATUS_SUCCEEDED}"
            )
            self.last_logged_status = nav_status
        
        if nav_status is None:
            self.feedback_message = "Waiting for navigation to start"
            return py_trees.common.Status.RUNNING
        
        if nav_status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f'[BT] Navigation succeeded! (after {self.update_count} updates)')
            self.feedback_message = "Navigation complete"
            # Reset status for next navigation
            self.blackboard.set('nav_status', None)
            return py_trees.common.Status.SUCCESS
        
        elif nav_status in [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED]:
            self.node.get_logger().error(f'[BT] Navigation failed with status: {nav_status}')
            self.feedback_message = f"Navigation failed: {nav_status}"
            # Reset status
            self.blackboard.set('nav_status', None)
            return py_trees.common.Status.FAILURE
        
        else:
            # Still running
            self.feedback_message = f"Navigation in progress (status={nav_status})"
            return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Called when behavior terminates"""
        self.node.get_logger().info(
            f'[BT] IsNavigationComplete: terminate() called with status={new_status} '
            f'after {self.update_count} updates'
        )
        self.last_logged_status = None


class IsManipulationComplete(py_trees.behaviour.Behaviour):
    """
    Check if manipulation is complete based on timeout or process status
    TODO
    """
    
    def __init__(self, name, node, blackboard, timeout=60.0):
        super().__init__(name)
        self.node = node
        self.blackboard = blackboard
        self.timeout = timeout
        self.start_time = None
    
    def initialise(self):
        """Initialize timer"""
        if self.start_time is None:
            self.start_time = self.node.get_clock().now()
            self.node.get_logger().info('[BT] Starting manipulation timer')
    
    def update(self):
        """Check condition"""
        # Check if processes are still running
        ik_process = self.blackboard.get('ik_process')
        mocap_process = self.blackboard.get('mocap_process')
        
        # If processes have terminated, manipulation is complete
        if ik_process and ik_process.poll() is not None:
            self.node.get_logger().info('[BT] IK process terminated')
            self.feedback_message = "IK process ended"
            self.start_time = None
            return py_trees.common.Status.SUCCESS
        
        if mocap_process and mocap_process.poll() is not None:
            self.node.get_logger().info('[BT] Mocap process terminated')
            self.feedback_message = "Mocap process ended"
            self.start_time = None
            return py_trees.common.Status.SUCCESS
        
        # Check timeout
        if self.start_time:
            elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > self.timeout:
                self.node.get_logger().info('[BT] Manipulation timeout reached')
                self.feedback_message = "Manipulation timeout"
                self.start_time = None
                return py_trees.common.Status.SUCCESS
        
        self.feedback_message = "Manipulation in progress"
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Reset timer"""
        self.start_time = None


class IsHumanDetected(py_trees.behaviour.Behaviour):
    """
    Check if human detection is available by monitoring /human_masks_dual topic.
    Returns SUCCESS if recent messages are being published (within timeout).
    Returns FAILURE if no recent detection (so Selector can try next child).
    """
    
    def __init__(self, name, node, blackboard, message_timeout=2.0):
        super().__init__(name)
        self.node = node
        self.blackboard = blackboard
        self.message_timeout = message_timeout  # Max age for "fresh" message
        self.last_msg_time = None
        self.subscription = None
    
    def setup(self, **kwargs):
        """Setup subscriber"""
        if self.subscription is None:
            self.subscription = self.node.create_subscription(
                Image,
                '/human_masks_dual',
                self._detection_callback,
                10
            )
            self.node.get_logger().info('[BT] IsHumanDetected: Subscribed to /human_masks_dual')
        return True
    
    def initialise(self):
        """Reset detection state"""
        self.node.get_logger().info('[BT] IsHumanDetected: Checking for human detection...')
        # Don't reset last_msg_time - we want to check if recent messages exist
    
    def update(self):
        """Check if recent human detection messages are available"""
        self.node.get_logger().debug('[BT] IsHumanDetected: update() called')
        if self.last_msg_time is None:
            self.feedback_message = "No human detection messages yet"
            # Return FAILURE so Selector tries the next child (rotation)
            return py_trees.common.Status.FAILURE
        
        # Check if message is recent enough
        current_time = self.node.get_clock().now()
        age = (current_time - self.last_msg_time).nanoseconds / 1e9
        
        if age < self.message_timeout:
            self.node.get_logger().info(f'[BT] Human detected! (message age: {age:.2f}s)')
            self.feedback_message = f"Human detected (fresh message)"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Detection stale ({age:.2f}s old)"
            # Return FAILURE so Selector tries the next child (rotation)
            return py_trees.common.Status.FAILURE
    
    def _detection_callback(self, msg):
        """Store timestamp of latest detection message"""
        self.last_msg_time = self.node.get_clock().now()
    
    def terminate(self, new_status):
        """Cleanup"""
        if new_status == py_trees.common.Status.SUCCESS:
            self.node.get_logger().info('[BT] IsHumanDetected: Detection confirmed, proceeding to manipulation')
