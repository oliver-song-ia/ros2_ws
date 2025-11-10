#!/usr/bin/env python3
"""
Mission Coordinator Node
Orchestrates navigation and manipulation tasks in sequence.

States:
    IDLE: Waiting for mission start
    NAVIGATION: Navigating to human pose
    MANIPULATION: Performing manipulation task
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatusArray, GoalStatus
from nav2_msgs.action import NavigateToPose
import subprocess
import time
import signal
import os


class MissionState(Enum):
    """State machine states"""
    IDLE = 0
    NAVIGATION = 1
    MANIPULATION = 2
    TRANSITION = 3  # Transitioning between modes
    ERROR = 4


class MissionCoordinator(Node):
    """
    Coordinates navigation and manipulation tasks.
    
    This node implements a state machine that:
    1. Waits for human pose and mission trigger (IDLE)
    2. Navigates robot to human location (NAVIGATION)
    3. Executes manipulation task (MANIPULATION)
    4. Returns to IDLE for next mission
    """
    
    def __init__(self):
        super().__init__('mission_coordinator')
        
        # Parameters
        self.declare_parameter('navigation_timeout', 120.0)
        self.declare_parameter('manipulation_timeout', 60.0)
        self.declare_parameter('navigation_goal_tolerance', 0.5)
        self.declare_parameter('max_navigation_retries', 3)
        self.declare_parameter('auto_start', False)
        
        self.nav_timeout = self.get_parameter('navigation_timeout').value
        self.manip_timeout = self.get_parameter('manipulation_timeout').value
        self.goal_tolerance = self.get_parameter('navigation_goal_tolerance').value
        self.max_nav_retries = self.get_parameter('max_navigation_retries').value
        self.auto_start = self.get_parameter('auto_start').value
        
        # State machine
        self.state = MissionState.IDLE
        self.previous_state = None
        self.nav_retry_count = 0
        
        # Mission data
        self.human_pose = None
        self.human_pose_received = False
        self.mission_triggered = False
        self.start_nav = False
        
        # Navigation action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal_handle = None
        
        # Navigation status tracking
        self.nav_status = None
        self.nav_feedback = None
        
        # Process handles for manipulation mode
        self.ik_process = None
        self.mocap_process = None
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/mission_status', 10)
        self.cmd_vel_stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.human_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # Can be from human detection or manual input
            self.human_pose_callback,
            10
        )
        
        self.mission_trigger_sub = self.create_subscription(
            Bool,
            '/mission_trigger',
            self.mission_trigger_callback,
            10
        )
        
        # Navigation action status subscriber
        self.nav_status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.nav_status_callback,
            10
        )
        
        # Navigation feedback subscriber
        self.nav_feedback_sub = self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.nav_feedback_callback,
            10
        )
        
        # Services
        self.start_mission_srv = self.create_service(
            Trigger,
            '/start_mission',
            self.start_mission_callback
        )
        
        self.abort_mission_srv = self.create_service(
            Trigger,
            '/abort_mission',
            self.abort_mission_callback
        )
        
        # State machine timer
        self.state_timer = self.create_timer(0.5, self.state_machine_update)
        
        # Timestamps
        self.state_start_time = self.get_clock().now()
        
        self.get_logger().info('Mission Coordinator initialized')
        self.get_logger().info(f'Auto-start: {self.auto_start}')
        
        if self.auto_start:
            self.get_logger().info('Waiting for human pose to auto-start mission...')
    
    def human_pose_callback(self, msg):
        """Store human pose when received"""
        self.human_pose = msg
        self.human_pose_received = True
        self.get_logger().info(
            f'Human pose received: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})'
        )
        
        # Auto-start if enabled
        if self.auto_start and self.state == MissionState.IDLE:
            self.mission_triggered = False
            self.get_logger().info('Auto-starting mission...')
            self.start_nav = True
            # self.transition_to_state(MissionState.NAVIGATION)
    
    def mission_trigger_callback(self, msg):
        """Handle mission trigger"""
        if msg.data:
            self.mission_triggered = True
            self.get_logger().info('Mission triggered via topic')
    
    def nav_status_callback(self, msg):
        """Handle navigation action status updates"""
        if msg.status_list:
            self.nav_status = msg.status_list[-1].status
        self.get_logger().info(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!Nav status updated: {self.nav_status}")
    
    def nav_feedback_callback(self, msg):
        """Handle navigation action feedback"""
        self.nav_feedback = msg.feedback
    
    def start_mission_callback(self, request, response):
        """Service callback to start mission"""
        if self.state != MissionState.IDLE:
            response.success = False
            response.message = f'Cannot start mission in state: {self.state.name}'
            return response
        
        if not self.human_pose_received:
            response.success = False
            response.message = 'No human pose available. Publish to /goal_pose first.'
            return response
        
        self.mission_triggered = True
        response.success = True
        response.message = 'Mission started'
        self.get_logger().info('Mission started via service call')
        return response
    
    def abort_mission_callback(self, request, response):
        """Service callback to abort mission"""
        # TODO: emergency stop and cleanup?
        self.get_logger().warn('Mission abort requested')
        self.transition_to_state(MissionState.ERROR)
        response.success = True
        response.message = 'Mission aborted'
        return response
    
    def transition_to_state(self, new_state):
        """Transition to a new state"""
        if new_state == self.state:
            return
        
        self.get_logger().info(f'State transition: {self.state.name} → {new_state.name}')
        self.previous_state = self.state
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        
        # Publish state
        msg = String()
        msg.data = new_state.name
        self.state_pub.publish(msg)
        
        # State entry actions
        if new_state == MissionState.NAVIGATION:
            self.enter_navigation_state()
        elif new_state == MissionState.MANIPULATION:
            self.enter_manipulation_state()
        elif new_state == MissionState.IDLE:
            self.enter_idle_state()
    
    def state_machine_update(self):
        """Main state machine update loop"""
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)
        
        if self.state == MissionState.IDLE:
            self.update_idle_state()
        
        elif self.state == MissionState.NAVIGATION:
            self.update_navigation_state()
        
        elif self.state == MissionState.MANIPULATION:
            self.update_manipulation_state()
        
        elif self.state == MissionState.ERROR:
            self.update_error_state()
    
    # ========== IDLE STATE ==========
    
    def enter_idle_state(self):
        """Entry actions for IDLE state"""
        self.mission_triggered = False
        self.nav_retry_count = 0
        self.get_logger().info('Entering IDLE state. Ready for next mission.')
    
    def update_idle_state(self):
        """Update IDLE state"""
        if self.mission_triggered and self.human_pose_received:
            self.transition_to_state(MissionState.NAVIGATION)
        
        if self.start_nav:
            self.get_logger().info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!Auto-starting navigation...")
            self.transition_to_state(MissionState.NAVIGATION)
            self.start_nav = False
        
        if self.mission_triggered and not self.human_pose_received:
            self.get_logger().warn('Mission triggered but no human pose available')
            self.get_logger().warn('Entering manipulation state without navigation')
            self.transition_to_state(MissionState.MANIPULATION)
    
    # ========== NAVIGATION STATE ==========
    
    def enter_navigation_state(self):
        """Entry actions for NAVIGATION state"""
        self.get_logger().info('Starting navigation to human pose...')
        
        # Reset navigation tracking
        self.nav_status = None
        self.nav_feedback = None
        
        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available!')
            self.transition_to_state(MissionState.ERROR)
            return
        
        # # Send navigation goal
        # goal_msg = NavigateToPose.Goal()
        # goal_msg.pose = self.human_pose
        
        # self.get_logger().info(f'Sending navigation goal to ({self.human_pose.pose.position.x:.2f}, {self.human_pose.pose.position.y:.2f})')
        
        # send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        # send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Handle navigation goal acceptance/rejection"""
        self.nav_goal_handle = future.result()
        if not self.nav_goal_handle.accepted:
            self.get_logger().error('Navigation goal was rejected!')
            self.transition_to_state(MissionState.ERROR)
            return
        
        self.get_logger().info('Navigation goal accepted')
    
    def update_navigation_state(self):
        """Update NAVIGATION state"""
        self.get_logger().info(f"nav update, {self.mission_triggered}")
        if self.mission_triggered:
            self.get_logger().info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!Mission triggered during navigation, transitioning to MANIPULATION...")
            self.transition_to_state(MissionState.MANIPULATION)
        
        # Check for timeout
        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        if elapsed > self.nav_timeout:
            self.get_logger().error('Navigation timeout!')
            self.handle_navigation_failure()
            return
        
        # Check if we have a goal handle yet
        if self.nav_goal_handle is None:
            return
        
        # Check navigation status from topic
        if self.nav_status is None:
            return
        
        # Log feedback if available
        if self.nav_feedback is not None:
            self.get_logger().info(
                f'Navigation in progress... '
                f'Distance remaining: {self.nav_feedback.distance_remaining:.2f}m',
                throttle_duration_sec=2.0
            )
        
        # Check if navigation is complete based on status
        if self.nav_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
            self.transition_to_state(MissionState.MANIPULATION)
        
        elif self.nav_status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation was canceled')
            self.handle_navigation_failure()
        
        elif self.nav_status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation was aborted')
            self.handle_navigation_failure()
    
    def handle_navigation_failure(self):
        """Handle navigation failure with retries"""
        self.nav_retry_count += 1
        
        if self.nav_retry_count < self.max_nav_retries:
            self.get_logger().info(
                f'!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Retrying navigation ({self.nav_retry_count}/{self.max_nav_retries})...'
            )
            # Cancel current goal if any
            if self.nav_goal_handle is not None:
                self.nav_goal_handle.cancel_goal_async()
            
            # Schedule retry after delay using timer (non-blocking)
            self.create_timer(1.0, lambda: self.transition_to_state(MissionState.NAVIGATION), once=True)
        else:
            self.get_logger().error('Max navigation retries reached. Mission failed.')
            self.transition_to_state(MissionState.ERROR)
    
    # ========== MANIPULATION STATE ==========
    
    def enter_manipulation_state(self):
        """Entry actions for MANIPULATION state"""
        self.get_logger().info('Starting manipulation task...')
        
        # Stop the robot
        # self.stop_robot_motion()
        
        # Start IK controller
        self.start_ik_controller()
        
        # Start mocap publisher
        self.start_mocap_publisher()
    
    def update_manipulation_state(self):
        """Update MANIPULATION state"""
        
        # Check for timeout
        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        if elapsed > self.manip_timeout:
            self.get_logger().info('Manipulation timeout reached. Task complete.')
            self.transition_to_state(MissionState.IDLE)
            return
        
        # Check if processes are still running
        if self.ik_process and self.ik_process.poll() is not None:
            self.get_logger().info('IK controller process terminated')
            self.cleanup_manipulation()
            self.transition_to_state(MissionState.IDLE)
            return
        
        if self.mocap_process and self.mocap_process.poll() is not None:
            self.get_logger().info('Mocap publisher process terminated')
            self.cleanup_manipulation()
            self.transition_to_state(MissionState.IDLE)
            return
    
    def start_ik_controller(self):
        """Start the mobile IK controller"""
        try:
            self.get_logger().info('Launching IK controller...')
            self.ik_process = subprocess.Popen(
                ['python3', 'src/ia_robot/src/kinematics/mobile_ik_controller.py'],
                cwd=os.path.expanduser('~/code/ia_robot_sim'),
                # stdout=subprocess.PIPE,
                # stderr=subprocess.PIPE
            )
            self.get_logger().info(f'IK controller started (PID: {self.ik_process.pid})')
        except Exception as e:
            self.get_logger().error(f'Failed to start IK controller: {e}')
            self.transition_to_state(MissionState.ERROR)
    
    def start_mocap_publisher(self):
        """Start the mocap data publisher"""
        try:
            self.get_logger().info('Launching mocap publisher...')
            # Adjust the command based on your mocap publisher arguments
            self.mocap_process = subprocess.Popen(
                [
                    'python3',
                    'src/genesis_recorder/src/ros_mocap_publisher.py',
                    '--batch',
                    '--ros'
                ],
                cwd=os.path.expanduser('~/code/ia_robot_sim'),
                # stdout=subprocess.PIPE,
                # stderr=subprocess.PIPE
            )
            self.get_logger().info(f'Mocap publisher started (PID: {self.mocap_process.pid})')
        except Exception as e:
            self.get_logger().error(f'Failed to start mocap publisher: {e}')
            self.transition_to_state(MissionState.ERROR)
    
    def cleanup_manipulation(self):
        """Clean up manipulation processes"""
        if self.ik_process:
            self.get_logger().info('Stopping IK controller...')
            self.ik_process.terminate()
            try:
                self.ik_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.ik_process.kill()
            self.ik_process = None
        
        if self.mocap_process:
            self.get_logger().info('Stopping mocap publisher...')
            self.mocap_process.terminate()
            try:
                self.mocap_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.mocap_process.kill()
            self.mocap_process = None
    
    # ========== ERROR STATE ==========
    
    def update_error_state(self):
        """Update ERROR state"""
        # Clean up everything
        self.cleanup_manipulation()
        self.stop_robot_motion()
        
        # Cancel any active navigation
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
        
        # Return to IDLE after cleanup
        self.get_logger().info('Error cleanup complete. Returning to IDLE.')
        self.state = MissionState.IDLE
        self.mission_triggered = False
    
    # ========== UTILITIES ==========
    
    def stop_robot_motion(self):
        """Send zero velocity command to stop robot"""
        msg = Twist()
        # Send stop command multiple times without blocking
        for _ in range(5):
            self.cmd_vel_stop_pub.publish(msg)
        self.get_logger().info('Robot motion stopped')
    
    def destroy_node(self):
        """Cleanup on node shutdown"""
        self.get_logger().info('Shutting down mission coordinator...')
        self.cleanup_manipulation()
        # Don't call stop_robot_motion during shutdown to avoid publisher errors
        # The navigation stack will handle stopping the robot
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    coordinator = MissionCoordinator()
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()