#!/usr/bin/env python3
"""
State handlers for Mission Coordinator
Implements the State pattern to separate state-specific logic.
"""

import subprocess
import os
from abc import ABC, abstractmethod
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class StateHandler(ABC):
    """Base class for all state handlers"""
    
    def __init__(self, coordinator):
        """
        Initialize state handler.
        
        Args:
            coordinator: Reference to the MissionCoordinator node
        """
        self.coordinator = coordinator
        self.logger = coordinator.get_logger()
    
    @abstractmethod
    def enter(self):
        """Called when entering this state"""
        pass
    
    @abstractmethod
    def update(self):
        """Called periodically while in this state"""
        pass
    
    def exit(self):
        """Called when exiting this state (optional override)"""
        pass


class IdleStateHandler(StateHandler):
    """Handles IDLE state - waiting for mission trigger"""
    
    def enter(self):
        """Entry actions for IDLE state"""
        self.coordinator.mission_triggered = False
        self.coordinator.nav_retry_count = 0
        self.logger.info('Entering IDLE state. Ready for next mission.')
    
    def update(self):
        """Update IDLE state"""
        if self.coordinator.mission_triggered and self.coordinator.human_pose_received:
            self.coordinator.transition_to_state_enum(self.coordinator.MissionState.NAVIGATION)
        
        if self.coordinator.mission_triggered and not self.coordinator.human_pose_received:
            self.logger.warning('Mission triggered but no human pose available')
            self.logger.warning('Entering manipulation state without navigation')
            self.coordinator.transition_to_state_enum(self.coordinator.MissionState.MANIPULATION)


class NavigationStateHandler(StateHandler):
    """Handles NAVIGATION state - navigating to target pose"""
    
    def __init__(self, coordinator):
        super().__init__(coordinator)
        self.nav_goal_handle = None
        self.nav_status = None
        self.nav_feedback = None
    
    def enter(self):
        """Entry actions for NAVIGATION state"""
        self.logger.info('Starting navigation to human pose...')
        
        # Reset navigation tracking
        self.nav_status = None
        self.nav_feedback = None
        self.nav_goal_handle = None
        
        # Wait for action server
        if not self.coordinator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.logger.error('Navigation action server not available!')
            self.coordinator.transition_to_state_enum(self.coordinator.MissionState.ERROR)
            return
        
        # # Send navigation goal
        # goal_msg = NavigateToPose.Goal()
        # goal_msg.pose = self.coordinator.human_pose
        
        # self.logger.info(
        #     f'Sending navigation goal to '
        #     f'({self.coordinator.human_pose.pose.position.x:.2f}, '
        #     f'{self.coordinator.human_pose.pose.position.y:.2f})'
        # )
        
        # send_goal_future = self.coordinator.nav_to_pose_client.send_goal_async(goal_msg)
        # send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Handle navigation goal acceptance/rejection"""
        self.nav_goal_handle = future.result()
        if not self.nav_goal_handle.accepted:
            self.logger.error('Navigation goal was rejected!')
            self.coordinator.transition_to_state_enum(self.coordinator.MissionState.ERROR)
            return
        
        self.logger.info('Navigation goal accepted')
    
    def update(self):
        """Update NAVIGATION state"""
        # Check for timeout
        elapsed = (
            self.coordinator.get_clock().now() - 
            self.coordinator.state_start_time
        ).nanoseconds / 1e9
        self.logger.info(f'Navigation update')
        
        if elapsed > self.coordinator.nav_timeout:
            self.logger.error('Navigation timeout!')
            self.handle_navigation_failure()
            return
        
        # Check if we have a goal handle yet
        if self.nav_goal_handle is None:
            return
        
        # Check navigation status from coordinator's tracking
        if self.coordinator.nav_status is None:
            return
        
        # Log feedback if available
        if self.coordinator.nav_feedback is not None:
            self.logger.info(
                f'Navigation in progress... '
                f'Distance remaining: {self.coordinator.nav_feedback.distance_remaining:.2f}m',
                throttle_duration_sec=2.0
            )
        
        # Check if navigation is complete based on status
        if self.coordinator.nav_status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.info('Navigation succeeded!')
            self.coordinator.transition_to_state_enum(self.coordinator.MissionState.MANIPULATION)
        
        elif self.coordinator.nav_status == GoalStatus.STATUS_CANCELED:
            self.logger.warning('Navigation was canceled')
            self.handle_navigation_failure()
        
        elif self.coordinator.nav_status == GoalStatus.STATUS_ABORTED:
            self.logger.error('Navigation was aborted')
            self.handle_navigation_failure()

        if self.coordinator.mission_triggered:
            self.coordinator.transition_to_state_enum(self.coordinator.MissionState.MANIPULATION)
            self.coordinator.mission_triggered = False
    
    def handle_navigation_failure(self):
        """Handle navigation failure with retries"""
        self.coordinator.nav_retry_count += 1
        
        if self.coordinator.nav_retry_count < self.coordinator.max_nav_retries:
            self.logger.info(
                f'Retrying navigation '
                f'({self.coordinator.nav_retry_count}/{self.coordinator.max_nav_retries})...'
            )
            # Cancel current goal if any
            if self.nav_goal_handle is not None:
                self.nav_goal_handle.cancel_goal_async()
            
            # Schedule retry after delay using timer (non-blocking)
            self.coordinator.create_timer(
                1.0,
                lambda: self.coordinator.transition_to_state_enum(
                    self.coordinator.MissionState.NAVIGATION
                ),
                once=True
            )
        else:
            self.logger.error('Max navigation retries reached. Mission failed.')
            self.coordinator.transition_to_state_enum(self.coordinator.MissionState.ERROR)
    
    def exit(self):
        """Cleanup when exiting navigation state"""
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()


class ManipulationStateHandler(StateHandler):
    """Handles MANIPULATION state - performing manipulation tasks"""
    
    def __init__(self, coordinator):
        super().__init__(coordinator)
        self.ik_process = None
        self.mocap_process = None
    
    def enter(self):
        """Entry actions for MANIPULATION state"""
        self.logger.info('Starting manipulation task...')
        
        # Start IK controller
        self.start_ik_controller()
        
        # Start mocap publisher
        self.start_mocap_publisher()
    
    def update(self):
        """Update MANIPULATION state"""
        # Check for timeout
        elapsed = (
            self.coordinator.get_clock().now() - 
            self.coordinator.state_start_time
        ).nanoseconds / 1e9
        
        if elapsed > self.coordinator.manip_timeout:
            self.logger.info('Manipulation timeout reached. Task complete.')
            self.coordinator.transition_to_state_enum(self.coordinator.MissionState.IDLE)
            return
        
        # Check if processes are still running
        if self.ik_process and self.ik_process.poll() is not None:
            self.logger.info('IK controller process terminated')
            self.cleanup_manipulation()
            self.coordinator.transition_to_state_enum(self.coordinator.MissionState.IDLE)
            return
        
        if self.mocap_process and self.mocap_process.poll() is not None:
            self.logger.info('Mocap publisher process terminated')
            self.cleanup_manipulation()
            self.coordinator.transition_to_state_enum(self.coordinator.MissionState.IDLE)
            return
    
    def start_ik_controller(self):
        """Start the mobile IK controller"""
        try:
            self.logger.info('Launching IK controller...')
            self.ik_process = subprocess.Popen(
                ['python3', 'src/ia_robot/src/mobile_ik_controller.py'],
                cwd=os.path.expanduser('~/code/ia_robot_sim'),
            )
            self.logger.info(f'IK controller started (PID: {self.ik_process.pid})')
        except Exception as e:
            self.logger.error(f'Failed to start IK controller: {e}')
            self.coordinator.transition_to_state_enum(self.coordinator.MissionState.ERROR)
    
    def start_mocap_publisher(self):
        """Start the mocap data publisher"""
        try:
            self.logger.info('Launching mocap publisher...')
            self.mocap_process = subprocess.Popen(
                [
                    'python3',
                    'src/genesis_recorder/src/ros_mocap_publisher.py',
                    '--batch',
                    '--ros'
                ],
                cwd=os.path.expanduser('~/code/ia_robot_sim'),
            )
            self.logger.info(f'Mocap publisher started (PID: {self.mocap_process.pid})')
        except Exception as e:
            self.logger.error(f'Failed to start mocap publisher: {e}')
            self.coordinator.transition_to_state_enum(self.coordinator.MissionState.ERROR)
    
    def cleanup_manipulation(self):
        """Clean up manipulation processes"""
        if self.ik_process:
            self.logger.info('Stopping IK controller...')
            self.ik_process.terminate()
            try:
                self.ik_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.ik_process.kill()
            self.ik_process = None
        
        if self.mocap_process:
            self.logger.info('Stopping mocap publisher...')
            self.mocap_process.terminate()
            try:
                self.mocap_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.mocap_process.kill()
            self.mocap_process = None
    
    def exit(self):
        """Cleanup when exiting manipulation state"""
        self.cleanup_manipulation()


class ErrorStateHandler(StateHandler):
    """Handles ERROR state - cleanup and recovery"""
    
    def enter(self):
        """Entry actions for ERROR state"""
        self.logger.error('Entering ERROR state. Performing cleanup...')
    
    def update(self):
        """Update ERROR state"""
        # Clean up everything
        self.cleanup_all()
        
        # Return to IDLE after cleanup
        self.logger.info('Error cleanup complete. Returning to IDLE.')
        self.coordinator.state = self.coordinator.MissionState.IDLE
        self.coordinator.mission_triggered = False
    
    def cleanup_all(self):
        """Perform all cleanup operations"""
        # Stop robot motion
        self.stop_robot_motion()
        
        # Cancel any active navigation
        if self.coordinator.nav_goal_handle is not None:
            self.coordinator.nav_goal_handle.cancel_goal_async()
            self.coordinator.nav_goal_handle = None
        
        # Cleanup manipulation processes through the handler
        if hasattr(self.coordinator, 'state_handlers'):
            manip_handler = self.coordinator.state_handlers.get(
                self.coordinator.MissionState.MANIPULATION
            )
            if manip_handler and isinstance(manip_handler, ManipulationStateHandler):
                manip_handler.cleanup_manipulation()
    
    def stop_robot_motion(self):
        """Send zero velocity command to stop robot"""
        msg = Twist()
        # Send stop command multiple times without blocking
        for _ in range(5):
            self.coordinator.cmd_vel_stop_pub.publish(msg)
        self.logger.info('Robot motion stopped')
