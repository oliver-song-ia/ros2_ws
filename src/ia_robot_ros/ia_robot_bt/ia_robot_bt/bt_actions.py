#!/usr/bin/env python3
"""
Behavior Tree Action Nodes
"""

import py_trees
import subprocess
import os
import signal
from geometry_msgs.msg import Twist


class StartManipulationAction(py_trees.behaviour.Behaviour):
    """
    Action to start manipulation processes
    """

    def __init__(self, name, node, blackboard, python_interpreter='python'):
        super().__init__(name)
        self.node = node
        self.blackboard = blackboard
        self.python_interpreter = python_interpreter
        self.ik_process = None
        self.controller_process = None
        self.mocap_process = None

        # Get workspace root from COLCON_PREFIX_PATH environment variable
        # COLCON_PREFIX_PATH points to <workspace>/install, so we need the parent
        colcon_prefix = os.environ.get('COLCON_PREFIX_PATH', '')
        if colcon_prefix:
            # COLCON_PREFIX_PATH can be a colon-separated list, take the first one
            first_prefix = colcon_prefix.split(':')[0]
            self.workspace_root = os.path.dirname(first_prefix)
        else:
            # Fallback to hardcoded path if environment variable not set
            self.workspace_root = os.path.expanduser('~/code/ia_robot_sim')
            self.node.get_logger().warn(
                '[BT]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! COLCON_PREFIX_PATH not set, using fallback workspace path: '
                f'{self.workspace_root}'
            )
    
    def setup(self, **kwargs):
        """Setup"""
        self.node.get_logger().info(f'[BT] Setting up {self.name}')
        return True
    
    def initialise(self):
        """Start manipulation processes"""
        self.node.get_logger().info(f'[BT] Initializing {self.name}')
        
        # Stop robot motion first
        self._stop_robot_motion()
        
        # Start all manipulation processes
        self._start_ik_controller()
        self._start_controller()
        self._start_mocap_publisher()
        
        # Store process handles in blackboard
        self.blackboard.set('ik_process', self.ik_process)
        self.blackboard.set('controller_process', self.controller_process)
        self.blackboard.set('mocap_process', self.mocap_process)
        self.blackboard.set('manipulation_active', True)
        
        self.feedback_message = "Manipulation started"
    
    def update(self):
        """Check if processes started successfully"""
        processes = {
            'IK controller': self.ik_process,
            'Controller': self.controller_process,
            'Mocap publisher': self.mocap_process
        }
        
        # Check if all processes started
        if not all(processes.values()):
            self.node.get_logger().error('[BT] Not all manipulation processes started')
            return py_trees.common.Status.FAILURE
        
        # Check if any process has died
        for name, process in processes.items():
            if process.poll() is not None:
                self.node.get_logger().error(f'[BT] {name} failed to start or crashed')
                return py_trees.common.Status.FAILURE
        
        self.node.get_logger().info('[BT] All manipulation processes started successfully')
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Cleanup"""
        self.node.get_logger().info(f'[BT] {self.name} terminated with status: {new_status}')
    
    def _stop_robot_motion(self):
        """Stop robot motion"""
        cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        msg = Twist()
        for _ in range(5):
            cmd_vel_pub.publish(msg)
        self.node.get_logger().info('[BT] Robot motion stopped')
    
    def _start_ik_controller(self):
        """Start IK controller process"""
        try:
            self.node.get_logger().info('[BT] Launching IK controller...')
            self.ik_process = subprocess.Popen(
                [self.python_interpreter, 'src/ia_robot_ros/ia_robot/src/kinematics/mobile_ik_controller.py'],
                cwd=self.workspace_root,
                start_new_session=True,  # Create new process group
            )
            self.node.get_logger().info(f'[BT] IK controller started (PID: {self.ik_process.pid})')
        except Exception as e:
            self.node.get_logger().error(f'[BT] Failed to start IK controller: {e}')
            self.ik_process = None
    
    def _start_controller(self):
        """Start robot controller process"""
        try:
            self.node.get_logger().info('[BT] Launching robot controller...')
            self.controller_process = subprocess.Popen(
                [self.python_interpreter, 'src/ia_robot_ros/ia_robot_planner/robot_controller.py'],
                cwd=self.workspace_root,
                start_new_session=True,
            )
            self.node.get_logger().info(f'[BT] Robot controller started (PID: {self.controller_process.pid})')
        except Exception as e:
            self.node.get_logger().error(f'[BT] Failed to start robot controller: {e}')
            self.controller_process = None
    
    def _start_mocap_publisher(self):
        """Start mocap publisher process"""
        try:
            self.node.get_logger().info('[BT] Launching mocap publisher...')
            self.mocap_process = subprocess.Popen(
                [
                    self.python_interpreter,
                    'src/genesis_recorder/src/ros_mocap_publisher.py',
                    '--mode-listener',
                    '--ros'
                ],
                cwd=self.workspace_root,
                start_new_session=True,  # Create new process group
            )
            self.node.get_logger().info(f'[BT] Mocap publisher started (PID: {self.mocap_process.pid})')
        except Exception as e:
            self.node.get_logger().error(f'[BT] Failed to start mocap publisher: {e}')
            self.mocap_process = None


class StopManipulationAction(py_trees.behaviour.Behaviour):
    """
    Action to stop manipulation processes
    """
    
    def __init__(self, name, node, blackboard):
        super().__init__(name)
        self.node = node
        self.blackboard = blackboard
    
    def setup(self, **kwargs):
        """Setup"""
        self.node.get_logger().info(f'[BT] Setting up {self.name}')
        return True
    
    def initialise(self):
        """Stop manipulation processes"""
        self.node.get_logger().info(f'[BT] Initializing {self.name}')
    
    def update(self):
        """Stop all processes"""
        # Get all process handles
        processes = {
            'IK controller': self.blackboard.get('ik_process'),
            'Controller': self.blackboard.get('controller_process'),
            'Mocap publisher': self.blackboard.get('mocap_process')
        }
        
        # Stop each process
        for name, process in processes.items():
            if process and process.poll() is None:
                self.node.get_logger().info(f'[BT] Stopping {name} process group...')
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    process.wait(timeout=3)
                except Exception as e:
                    self.node.get_logger().warn(f'[BT] {name} did not terminate, force killing: {e}')
                    try:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                        process.wait(timeout=2)
                    except Exception:
                        pass
        
        # Clear blackboard
        self.blackboard.set('ik_process', None)
        self.blackboard.set('controller_process', None)
        self.blackboard.set('mocap_process', None)
        self.blackboard.set('manipulation_active', False)
        
        self.node.get_logger().info('[BT] All manipulation processes stopped')
        self.feedback_message = "Manipulation stopped"
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Cleanup"""
        self.node.get_logger().info(f'[BT] {self.name} terminated with status: {new_status}')


class RotateToDetectAction(py_trees.behaviour.Behaviour):
    """
    Action to rotate the robot while checking for human detection.
    This behavior works in conjunction with IsHumanDetected condition.
    It continuously rotates until the parent sequence determines detection is successful.
    """
    
    def __init__(self, name, node, blackboard, rotation_speed=0.3):
        super().__init__(name)
        self.node = node
        self.blackboard = blackboard
        self.rotation_speed = rotation_speed  # rad/s (positive = counter-clockwise)
        self.cmd_vel_pub = None
    
    def setup(self, **kwargs):
        """Setup publisher"""
        if self.cmd_vel_pub is None:
            self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel_nav', 10)
            self.node.get_logger().info('[BT] RotateToDetect: Created /cmd_vel_nav publisher')
        return True
    
    def initialise(self):
        """Start rotation"""
        self.node.get_logger().info('[BT] RotateToDetect: Starting rotation to search for human...')
        self.feedback_message = "Rotating to detect human"
    
    def update(self):
        """Keep rotating - return RUNNING to keep action active"""
        # Send rotation command
        twist = Twist()
        twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(twist)
        
        # Always return RUNNING - the parent Selector will determine when to stop
        # based on the IsHumanDetected condition
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Stop rotation when behavior ends"""
        self.node.get_logger().info(f'[BT] RotateToDetect: Stopping rotation (status: {new_status})')
        
        # Stop the robot
        if self.cmd_vel_pub:
            twist = Twist()  # Zero velocity
            for _ in range(5):
                self.cmd_vel_pub.publish(twist)
        
        self.feedback_message = "Rotation stopped"


