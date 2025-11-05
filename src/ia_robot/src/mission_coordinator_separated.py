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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from enum import Enum
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatusArray, GoalStatus
from nav2_msgs.action import NavigateToPose

# Import state handlers
from mission_states import (
    IdleStateHandler,
    NavigationStateHandler,
    ManipulationStateHandler,
    ErrorStateHandler
)


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
        
        # Callback groups for better thread control
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_callback_group = ReentrantCallbackGroup()
        
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
        
        # Navigation action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal_handle = None
        
        # Navigation status tracking (shared with state handlers)
        self.nav_status = None
        self.nav_feedback = None
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/mission_status', 10)
        self.cmd_vel_stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State handlers - initialize after all attributes are set
        self.MissionState = MissionState  # Make enum accessible to handlers
        self.state_handlers = {
            MissionState.IDLE: IdleStateHandler(self),
            MissionState.NAVIGATION: NavigationStateHandler(self),
            MissionState.MANIPULATION: ManipulationStateHandler(self),
            MissionState.ERROR: ErrorStateHandler(self)
        }
        
        # Subscribers - use reentrant callback group to allow concurrent execution
        self.human_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # Can be from human detection or manual input
            self.human_pose_callback,
            10,
            callback_group=self.subscriber_callback_group
        )
        
        self.mission_trigger_sub = self.create_subscription(
            Bool,
            '/mission_trigger',
            self.mission_trigger_callback,
            10,
            callback_group=self.subscriber_callback_group
        )
        
        # Navigation action status subscriber
        self.nav_status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.nav_status_callback,
            10,
            callback_group=self.subscriber_callback_group
        )
        
        # Navigation feedback subscriber
        self.nav_feedback_sub = self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.nav_feedback_callback,
            10,
            callback_group=self.subscriber_callback_group
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
        
        # State machine timer - use dedicated callback group for consistent timing
        self.state_timer = self.create_timer(
            0.05, 
            self.state_machine_update,
            callback_group=self.timer_callback_group
        )
        
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
            self.mission_triggered = True
            self.get_logger().info('Auto-starting mission...')
    
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
        # TODO: switch mission from msg
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
        
        # Exit current state
        if self.state in self.state_handlers:
            self.state_handlers[self.state].exit()
        
        # Update state
        self.previous_state = self.state
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        
        # Publish state
        msg = String()
        msg.data = new_state.name
        self.state_pub.publish(msg)
        
        # Enter new state
        if new_state in self.state_handlers:
            self.state_handlers[new_state].enter()
    
    def transition_to_state_enum(self, new_state):
        """Helper method for state handlers to transition states"""
        self.transition_to_state(new_state)
    
    def state_machine_update(self):
        """Main state machine update loop"""
        # Publish current state (for monitoring tools)
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)
        
        # Only log state changes, not every cycle (reduces spam and overhead)
        # If you need to debug timing, use: self.get_logger().debug(f'State: {self.state.name}')
        
        # Delegate to current state handler
        if self.state in self.state_handlers:
            self.state_handlers[self.state].update()
    
    def destroy_node(self):
        """Cleanup on node shutdown"""
        # TODO: does nav state need shutdown/cancel goal?
        self.get_logger().info('Shutting down mission coordinator...')
        
        # Cleanup manipulation processes through the handler
        if MissionState.MANIPULATION in self.state_handlers:
            manip_handler = self.state_handlers[MissionState.MANIPULATION]
            if hasattr(manip_handler, 'cleanup_manipulation'):
                manip_handler.cleanup_manipulation()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    coordinator = MissionCoordinator()
    
    # Use MultiThreadedExecutor to allow callbacks to run concurrently
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(coordinator)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
