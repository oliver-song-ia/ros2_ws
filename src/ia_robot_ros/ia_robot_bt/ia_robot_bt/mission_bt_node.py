#!/usr/bin/env python3
"""
Mission Behavior Tree Node - Simplified with Integrated Planner

This node implements a simplified mission coordinator using behavior trees.
The robot planner state machine (rest/approach/assist/retreat) is now integrated
as a behavior tree subtree instead of a separate subprocess.

Simplified Workflow:
    1. Listen to /goal_pose topic
    2. When goal received, Nav2 handles navigation automatically
    3. Monitor Nav2 status topic for completion
    4. On navigation success, enter detection state:
       - Check if /human_masks_dual has recent messages
       - If yes, proceed to manipulation
       - If no, rotate robot until human is detected
    5. Start manipulation (IK, controller, mocap processes)
    6. Planner subtree manages robot modes during manipulation:
       - rest: Robot at rest position
       - approach: Move to position in front of human
       - assist: Follow shoulder targets
       - retreat: Return to behind human
       - Mode transitions via /set_mode topic
    7. Wait for manipulation complete (timeout or process termination)
    8. Stop manipulation and return to waiting for next goal

Behavior Tree Structure:
    Root (Sequence - loops forever)
    ├── Wait for Goal Pose
    ├── Wait for Navigation Complete (monitor Nav2 status)
    ├── Human Detection (Selector)
    │   ├── Is Human Detected? (check /human_masks_dual)
    │   └── Rotate to Detect (rotate until human found)
    ├── Start Manipulation (launch processes)
    ├── Manipulation Phase (Parallel)
    │   ├── Planner Subtree (mode state machine as BT)
    │   └── Wait for Manipulation Complete
    └── Stop Manipulation (cleanup)
"""

import rclpy
from rclpy.node import Node
import py_trees
from py_trees.trees import BehaviourTree
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.decorators import FailureIsSuccess, SuccessIsFailure, Repeat
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray
import signal
import os

from .bt_actions import (
    StartManipulationAction,
    StopManipulationAction,
    RotateToDetectAction,
)

from .bt_conditions import (
    IsGoalPoseReceived,
    IsNavigationComplete,
    IsManipulationComplete,
    IsHumanDetected,
)

from .planner_bt import create_planner_subtree


class MissionBehaviorTree(Node):
    """
    Simplified mission coordinator using behavior trees
    """
    
    def __init__(self):
        super().__init__('mission_bt_node')
        
        # Parameters
        self.declare_parameter('manipulation_timeout', 600.0)
        self.declare_parameter('tick_rate', 10.0)  # Hz
        self.declare_parameter('detection_timeout', 2.0)  # Max age for fresh detection message
        self.declare_parameter('rotation_speed', -0.3)  # rad/s for searching. negative for clockwise
        self.declare_parameter('print_tree', False)  # Print tree to console each tick
        self.declare_parameter('print_blackboard', False)  # Print blackboard each tick
        self.declare_parameter('python_interpreter', '~/miniforge3/envs/genesis/bin/python')  # Python executable path

        self.manip_timeout = self.get_parameter('manipulation_timeout').value
        self.tick_rate = self.get_parameter('tick_rate').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.print_tree = self.get_parameter('print_tree').value
        self.print_blackboard = self.get_parameter('print_blackboard').value
        self.python_interpreter = os.path.expanduser(self.get_parameter('python_interpreter').value)
        
        # Blackboard for sharing data between behaviors
        self.blackboard = py_trees.blackboard.Client(name="MissionBT")
        self.blackboard.register_key(key='goal_received', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='nav_status', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='manipulation_active', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='ik_process', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='controller_process', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='mocap_process', access=py_trees.common.Access.WRITE)
        
        # Initialize blackboard values
        self.blackboard.set('goal_received', False)
        self.blackboard.set('nav_status', None)
        self.blackboard.set('manipulation_active', False)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        
        # Subscribers
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        
        # Subscribe to Nav2 action status
        self.nav_status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.nav_status_callback,
            10
        )
        
        # Create behavior tree
        self.tree = self.create_behavior_tree()
        
        # Setup tree
        try:
            self.tree.setup(timeout=15)
            self.get_logger().info('Behavior tree setup complete')
        except Exception as e:
            self.get_logger().error(f'Failed to setup behavior tree: {e}')
            raise
        
        # Create timer to tick the tree
        timer_period = 1.0 / self.tick_rate
        self.tree_timer = self.create_timer(timer_period, self.tick_tree)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Mission Behavior Tree Node initialized (Simplified)')
        self.get_logger().info('='*60)
        self.get_logger().info('Workflow:')
        self.get_logger().info('  1. Waiting for goal on /goal_pose')
        self.get_logger().info('  2. Nav2 handles navigation automatically')
        self.get_logger().info('  3. Monitor Nav2 status for completion')
        self.get_logger().info('  4. Detection phase:')
        self.get_logger().info('     - Check /human_masks_dual for recent messages')
        self.get_logger().info('     - If not detected, rotate to search')
        self.get_logger().info('  5. Start manipulation on detection success')
        self.get_logger().info('  6. Planner subtree manages robot modes:')
        self.get_logger().info('     - rest/approach/assist/retreat')
        self.get_logger().info('     - Send mode commands to /set_mode topic')
        self.get_logger().info('  7. Wait and stop manipulation')
        self.get_logger().info('  8. Loop back to step 1')
        self.get_logger().info('='*60)
    
    def create_behavior_tree(self):
        """
        Create the simplified mission behavior tree
        
        Structure:
            Root (Sequence - repeats)
            ├── Wait for Goal Pose
            ├── Wait for Navigation Complete
            ├── Human Detection (Selector)
            │   ├── Is Human Detected? (SUCCESS if detected, FAILURE if not)
            │   └── Rotate to Detect (RUNNING while rotating)
            ├── Start Manipulation (start IK, controller, mocap processes)
            ├── Manipulation Phase (Parallel)
            │   ├── Planner Subtree (manages rest/approach/assist/retreat modes)
            │   └── Wait Manipulation Complete (timeout or process termination)
            └── Stop Manipulation (cleanup processes)
        
        The Planner Subtree (from planner_bt.py):
            - Selector with 4 mode sequences: rest, approach, assist, retreat
            - Each mode executes continuously (RUNNING) until mode change requested
            - Mode changes via /set_mode topic or automatic transitions
            - Publishes /target_eef_poses and /mode topics
        
        The Detection Selector works as follows:
        - First tries IsHumanDetected condition
        - If SUCCESS (human detected) → Selector succeeds, moves to manipulation
        - If FAILURE (no detection) → Tries RotateToDetectAction
        - RotateToDetectAction returns RUNNING and keeps rotating
        - On next tick (because Selector has memory=False):
          * Selector tries IsHumanDetected again from the start
          * If still no detection (FAILURE) → Tries RotateToDetectAction again (keeps rotating)
          * If detected (SUCCESS) → Selector succeeds, rotation stops
        """
        
        # Create behavior nodes
        wait_for_goal = IsGoalPoseReceived(
            "Wait for Goal Pose",
            self.blackboard
        )
        
        wait_nav_complete = IsNavigationComplete(
            "Wait for Navigation Complete",
            self,
            self.blackboard
        )
        
        # Detection phase - using Selector pattern
        # Selector tries children in order until one succeeds
        is_human_detected = IsHumanDetected(
            "Is Human Detected?",
            self,
            self.blackboard,
            message_timeout=self.detection_timeout
        )
        
        rotate_to_detect = RotateToDetectAction(
            "Rotate to Detect",
            self,
            self.blackboard,
            rotation_speed=self.rotation_speed
        )
        
        # Selector: First check if human detected, otherwise rotate
        # The selector will keep ticking until IsHumanDetected returns SUCCESS
        detection_phase = Selector(
            name="Human Detection Phase",
            memory=False,  # Re-evaluate from first child each tick
            children=[
                is_human_detected,     # If this succeeds, selector succeeds
                rotate_to_detect,      # If detection is RUNNING, this rotates (returns RUNNING)
            ]
        )
        
        start_manipulation = StartManipulationAction(
            "Start Manipulation",
            self,
            self.blackboard,
            python_interpreter=self.python_interpreter
        )
        
        # Create planner subtree that runs during manipulation
        planner_subtree = create_planner_subtree(self, self.blackboard)
        
        # Wrap planner in a sequence so it runs after manipulation starts
        # Use Parallel to run planner and wait condition concurrently
        manipulation_phase = Parallel(
            name="Manipulation Phase",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
            children=[
                planner_subtree,  # Runs continuously, managing robot planner states
                IsManipulationComplete(
                    "Wait Manipulation Complete",
                    self,
                    self.blackboard,
                    timeout=self.manip_timeout
                )
            ]
        )
        
        stop_manipulation = StopManipulationAction(
            "Stop Manipulation",
            self,
            self.blackboard
        )
        
        # Build the tree structure
        # Main mission sequence (repeats continuously)
        mission_sequence = Sequence(
            name="Mission Sequence",
            memory=True,  # Remember progress through children - don't restart from beginning each tick
            children=[
                wait_for_goal,
                wait_nav_complete,
                detection_phase,           # New detection state with rotation
                start_manipulation,
                manipulation_phase,        # Planner runs in parallel with timeout
                stop_manipulation,
            ]
        )
        
        # Wrap in a Repeat decorator to loop forever
        # When the sequence succeeds, it will reset and start again
        mission_loop = py_trees.decorators.Repeat(
            name="Mission Loop",
            child=mission_sequence,
            num_success=-1  # Repeat forever (-1 means infinite)
        )
        
        # Create and return the tree
        tree = BehaviourTree(mission_loop)
        
        return tree
    
    def tick_tree(self):
        """Tick the behavior tree"""
        try:
            self.tree.tick()
            
            # Print tree to console for debugging (optional - can be toggled with parameter)
            if hasattr(self, 'print_tree') and self.print_tree:
                print("\n" + py_trees.display.unicode_tree(
                    root=self.tree.root,
                    show_status=True
                ))
            
            # Log tree status periodically for debugging
            tip = self.tree.tip()
            if tip:
                status = tip.status
                # Only log when not idle to reduce spam
                if status != py_trees.common.Status.SUCCESS or self.blackboard.get('goal_received'):
                    self.get_logger().debug(
                        f'[BT] Tree tip: {tip.name}, status: {status}, '
                        f'feedback: {tip.feedback_message if hasattr(tip, "feedback_message") else "N/A"}'
                    )
            
            # Publish current status - use tree visualization
            status_msg = String()
            
            # Get the status based on current tree tip and blackboard
            tip_name = tip.name if tip else ""
            
            if self.blackboard.get('manipulation_active'):
                status_msg.data = "MANIPULATION"
            elif "Detection" in tip_name or "Rotate" in tip_name:
                status_msg.data = "DETECTION"
            elif self.blackboard.get('nav_status') is not None:
                status_msg.data = "NAVIGATION"
            elif self.blackboard.get('goal_received'):
                status_msg.data = "GOAL_RECEIVED"
            else:
                status_msg.data = "IDLE"
            
            # Generate the tree status visualization
            tree_status = py_trees.display.unicode_tree(
                root=self.tree.root,
                show_status=True
            )
            
            status_msg.data += "\n" + tree_status
            self.status_pub.publish(status_msg)
            
            # Print blackboard if requested
            if self.print_blackboard:
                print("\n" + "="*60)
                print("BLACKBOARD STATE:")
                print(f"  goal_received: {self.blackboard.get('goal_received')}")
                print(f"  nav_status: {self.blackboard.get('nav_status')}")
                print(f"  manipulation_active: {self.blackboard.get('manipulation_active')}")
                print("="*60)
            
        except Exception as e:
            self.get_logger().error(f'Error ticking tree: {e}')
    
    def goal_pose_callback(self, msg):
        """
        Handle goal pose from /goal_pose topic
        We don't republish it - Nav2 will receive it directly
        We just set a flag to transition the behavior tree
        """
        self.get_logger().info(
            f'[BT] Goal pose received: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})'
        )
        self.get_logger().info('[BT] Nav2 will handle navigation automatically')
        
        # Set flag for behavior tree
        self.blackboard.set('goal_received', True)
    
    def nav_status_callback(self, msg):
        """
        Monitor Nav2 navigation status
        """
        if msg.status_list:
            latest_status = msg.status_list[-1].status
            self.blackboard.set('nav_status', latest_status)

            self.get_logger().info(f'[BT] Received Nav2 status update {msg.status_list[-1].status}')
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down mission behavior tree...')
        
        # Cleanup any running processes BEFORE ROS context is invalidated
        ik_process = self.blackboard.get('ik_process')
        mocap_process = self.blackboard.get('mocap_process')
        
        if ik_process and ik_process.poll() is None:
            self.get_logger().info(f'Terminating IK process group (PID: {ik_process.pid})...')
            try:
                # Kill entire process group
                os.killpg(os.getpgid(ik_process.pid), signal.SIGTERM)
                ik_process.wait(timeout=3)
                self.get_logger().info('IK process terminated successfully')
            except Exception as e:
                self.get_logger().warn(f'IK process did not terminate ({e}), force killing...')
                try:
                    os.killpg(os.getpgid(ik_process.pid), signal.SIGKILL)
                    ik_process.wait(timeout=2)
                except Exception:
                    pass
        
        if mocap_process and mocap_process.poll() is None:
            self.get_logger().info(f'Terminating mocap process group (PID: {mocap_process.pid})...')
            try:
                # Kill entire process group
                os.killpg(os.getpgid(mocap_process.pid), signal.SIGTERM)
                mocap_process.wait(timeout=3)
                self.get_logger().info('Mocap process terminated successfully')
            except Exception as e:
                self.get_logger().warn(f'Mocap process did not terminate ({e}), force killing...')
                try:
                    os.killpg(os.getpgid(mocap_process.pid), signal.SIGKILL)
                    mocap_process.wait(timeout=2)
                except Exception:
                    pass
        
        self.get_logger().info('Mission behavior tree shutdown complete')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # Initialize py_trees logging
    py_trees.logging.level = py_trees.logging.Level.INFO
    
    mission_bt = MissionBehaviorTree()
    
    try:
        rclpy.spin(mission_bt)
    except KeyboardInterrupt:
        pass
    finally:
        mission_bt.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
