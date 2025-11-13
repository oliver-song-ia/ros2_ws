#!/usr/bin/env python3

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, PoseArray, Twist, Pose, Point, Quaternion, PointStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState, PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, Float64MultiArray, Float32MultiArray, String
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
from simple_mobile_ik import SimpleMobileIK
from signed_distance_field import SignedDistanceField
from collision_checker import CollisionChecker
from mobile_ik_config import MobileIKConfig
from transforms3d import quaternions
import threading
import time
import argparse
from ia_robot_interfaces.action import MoveToStandby
from ia_robot_interfaces.srv import SetJointLock

# set standby pose:
# ros2 action send_goal /move_to_standby ia_robot_interfaces/action/MoveToStandby "{}"

# lock joints:
# ros2 service call /set_joint_lock ia_robot_interfaces/srv/SetJointLock "{lock: true}"

# Standby configuration constants
STANDBY_NUM_FRAMES = 10  # Number of interpolation frames for standby sequence
STANDBY_FRAME_DELAY = 0.2  # Seconds between standby frames (5 * dt where dt=0.05)

# Standby target joint positions (CHEST1, ARM0-3_LEFT, ARM0-3_RIGHT)
# arm raised position
STARTUP_CHEST1 = 0.0
STARTUP_ARM0_LEFT = -0.027564312188662548
STARTUP_ARM1_LEFT = 1.6118096520764873
STARTUP_ARM2_LEFT = -1.7136484415829512
STARTUP_ARM3_LEFT = -0.027757140337728754
STARTUP_ARM0_RIGHT = -0.028090596963566316
STARTUP_ARM1_RIGHT = 1.614862432321618
STARTUP_ARM2_RIGHT = -1.7278826442179231
STARTUP_ARM3_RIGHT = -0.027757140337658377

# arm lowered position
STANDBY_CHEST1 = 0.0
STANDBY_ARM0_LEFT = 0.0
STANDBY_ARM1_LEFT = 0.0
STANDBY_ARM2_LEFT = 0.0
STANDBY_ARM3_LEFT = 0.0
STANDBY_ARM0_RIGHT = 0.0
STANDBY_ARM1_RIGHT = 0.0
STANDBY_ARM2_RIGHT = 0.0
STANDBY_ARM3_RIGHT = 0.0


class MobileIKController(Node):
    def __init__(self, use_ik=False):
        super().__init__('mobile_ik_controller')
        
        # Store standby mode
        self.use_ik = use_ik
        
        # Initialize the mobile IK solver
        self.get_logger().info("Initializing Mobile IK Controller...")
        self.ik_solver = SimpleMobileIK()
        
        # Initialize collision checker for obstacle avoidance
        self.collision_checker = CollisionChecker(self.ik_solver.model, None)  # Will set SDF later
        self.ik_solver.set_collision_checker(self.collision_checker)
        
        # Robot state
        self.current_q = np.zeros(self.ik_solver.n_joints)
        self.current_q[2] = np.pi  # Initial chassis orientation facing human (-x)
        self.previous_q = None
        self.previous_chassis_pose = (0.0, 0.0, 0.0)
        
        # Current pose from odometry
        self.current_odom_pose = None
        self.odom_lock = threading.Lock()
        
        # Velocity filtering for chassis commands
        self.velocity_history = {
            'vx': [],
            'vy': [],
            'wz': []
        }
        self.history_length = 5  # Number of samples for moving average
        self.velocity_threshold = 0.005  # Minimum velocity to publish (noise filter)
        
        # Target poses
        self.left_target = None
        self.right_target = None
        self.target_lock = threading.Lock()
        
        # Current mode (from /mode topic)
        self.current_mode = "rest"  # Default mode
        self.mode_lock = threading.Lock()
        
        # Obstacle avoidance with robot-centered bounds
        self.sdf = SignedDistanceField(
            bounds=MobileIKConfig.SDF_BOUNDS, 
            resolution=MobileIKConfig.SDF_RESOLUTION,
            max_distance=MobileIKConfig.SDF_MAX_DISTANCE,
            robot_centered=True  # Enable robot-centered dynamic bounds
        )
        self.obstacle_points = None
        self.obstacle_lock = threading.Lock()
        self.sdf_update_counter = 0  # For rate limiting SDF updates
        
        # Update collision checker with SDF
        self.collision_checker.sdf = self.sdf
        
        # TF2 setup for coordinate transformations with dedicated thread
        # Use a larger cache time and ensure proper buffer setup
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        
        # Create a separate executor and node for TF listener to run on its own thread
        # This prevents blocking from other callbacks in the main node
        self.tf_executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        self.tf_node = rclpy.create_node('tf_listener_node')
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.tf_node)
        
        # Start the dedicated TF thread
        self.tf_thread = threading.Thread(target=self._tf_thread_worker, daemon=True)
        self.tf_thread_running = True
        self.tf_thread.start()
        
        # Publishers
        joint_state_topic = '/joint_commands' if self.use_ik else '/ik/joint_states'
        self.joint_states_pub = self.create_publisher(JointState, joint_state_topic, 10)
        self.upper_body_commands_pub = self.create_publisher(Float64MultiArray, '/upper_body_controller/commands', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.fk_result_pub = self.create_publisher(PoseArray, '/ik/current_eef_poses', 10)  # For debugging
        self.current_poses_pub = self.create_publisher(PoseArray, '/current_poses', 10)  # ARM2_LEFT and ARM2_RIGHT poses
        
        # Visualization publishers for obstacle avoidance
        self.collision_distances_pub = self.create_publisher(Float32MultiArray, '/collision_distances', 10)
        self.sdf_markers_pub = self.create_publisher(MarkerArray, '/sdf_visualization', 10)
        self.obstacle_points_pub = self.create_publisher(PointCloud2, '/processed_obstacles', 10)
        
        # Subscribers  
        self.target_poses_sub = self.create_subscription(
            PoseArray,
            '/demo_target_poses',
            self.target_poses_callback,
            10
        )

        # Odometry subscriber for current pose
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Point cloud subscriber for obstacle avoidance
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/points_dep',
            self.pointcloud_callback,
            10
        )

        # Mode subscriber
        self.mode_sub = self.create_subscription(
            String,
            '/mode',
            self.mode_callback,
            10
        )

        # Set pose when new episode begins
        self.frame_info_sub = self.create_subscription(
            DiagnosticArray,
            '/mocap_frame_info',
            self.frame_info_callback,
            10
        )
        self.frame_idx_loop = float('inf')
        self.publish_pose = False

        # Get joint names from IK solver
        self.joint_names = self.ik_solver.get_joint_names()
        
        # Chassis joint names
        self.chassis_joint_names = ["chassis_x_joint", "chassis_y_joint", "chassis_rotation_joint"]
        self.upper_joint_names = ["CHEST1", "ARM0_LEFT", "ARM1_LEFT", "ARM2_LEFT", "ARM3_LEFT", "ARM0_RIGHT", "ARM1_RIGHT", "ARM2_RIGHT", "ARM3_RIGHT"]
        self.upper_joint_indices = [self.joint_names.index(name) for name in self.upper_joint_names]

        # Natural orientations will be calculated on demand, not cached
        
        # Standby sequence state
        self.standby_complete = False
        self.joints_locked = False  # Joint lock state
        self.standby_target = np.array([
            STANDBY_CHEST1,
            STANDBY_ARM0_LEFT,
            STANDBY_ARM1_LEFT,
            STANDBY_ARM2_LEFT,
            STANDBY_ARM3_LEFT,
            STANDBY_ARM0_RIGHT,
            STANDBY_ARM1_RIGHT,
            STANDBY_ARM2_RIGHT,
            STANDBY_ARM3_RIGHT
        ])
        self.startup_target = np.array([
            STARTUP_CHEST1,
            STARTUP_ARM0_LEFT,
            STARTUP_ARM1_LEFT,
            STARTUP_ARM2_LEFT,
            STARTUP_ARM3_LEFT,
            STARTUP_ARM0_RIGHT,
            STARTUP_ARM1_RIGHT,
            STARTUP_ARM2_RIGHT,
            STARTUP_ARM3_RIGHT
        ])
        
        # Standby action state (for action server)
        self.standby_action_active = False
        self.standby_action_progress = 0  # Current frame number
        self.standby_action_goal_handle = None
        
        # Create callback group for action server
        self.action_callback_group = ReentrantCallbackGroup()
        
        # Create action server for move to standby
        self._standby_action_server = ActionServer(
            self,
            MoveToStandby,
            'move_to_standby',
            execute_callback=self.execute_standby_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group
        )
        
        # Create service for joint lock
        self._joint_lock_service = self.create_service(
            SetJointLock,
            'set_joint_lock',
            self.handle_joint_lock,
            callback_group=self.action_callback_group
        )
        
        self.get_logger().info("Mobile IK Controller initialized successfully")
        self.get_logger().info("Natural orientations stored for target conversion")
        self.get_logger().info("Publishing standby interpolation sequence...")
        
        # Print obstacle avoidance configuration
        MobileIKConfig.print_config()

        self.dt = 0.05  # 20 Hz control loop
        
        # Publish standby sequence before starting control loop
        if self.use_ik:
            self.get_logger().info("Using IK standby sequence (--use-ik enabled)")
            self.publish_standby_sequence_ik()
        else:
            self.get_logger().info("Using standard standby sequence")
            self.publish_standby_sequence()
        
        # Control timer - start after startup sequence
        
        # TODO: read dt from config/ROS clock?
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("Waiting for target poses on /target_poses...")
    
    def _tf_thread_worker(self):
        """Worker function for the dedicated TF listener thread"""
        try:
            self.tf_executor.add_node(self.tf_node)
            while self.tf_thread_running and rclpy.ok():
                try:
                    self.tf_executor.spin_once(timeout_sec=0.1)
                except Exception as e:
                    self.get_logger().warn(f"TF thread error: {e}")
        except Exception as e:
            self.get_logger().error(f"TF thread worker failed: {e}")
        finally:
            if self.tf_node:
                self.tf_executor.remove_node(self.tf_node)
    
    def __del__(self):
        """Cleanup when the node is destroyed"""
        try:
            if hasattr(self, 'tf_thread_running'):
                self.tf_thread_running = False
            if hasattr(self, 'tf_thread') and self.tf_thread.is_alive():
                self.tf_thread.join(timeout=1.0)
            if hasattr(self, 'tf_node'):
                self.tf_node.destroy_node()
        except Exception as e:
            # Use print instead of logger since node might be destroyed
            print(f"Error during cleanup: {e}")
    
    def convert_target_orientation(self, target_orientation_matrix, arm='left'):
        """Convert target orientation from identity to robot's natural orientation with 180° correction"""
        # If target is close to identity matrix, use natural orientation with 180° rotation
        identity = np.eye(3)
        
        # Calculate rotation error between target and identity
        R_error = identity.T @ target_orientation_matrix
        trace_R = np.trace(R_error)
        trace_R = np.clip(trace_R, -1.0, 3.0)
        angle_error = np.arccos((trace_R - 1) / 2)
        
        # If target is close to identity (within 10 degrees), calculate current natural orientation
        if angle_error < np.radians(10):
            # Get current natural orientation (not cached) 
            q_zero = np.zeros(22)
            if arm == 'left':
                current_fk = self.ik_solver.forward_kinematics(q_zero, 'left')
                return current_fk[:3, :3]
            else:
                current_fk = self.ik_solver.forward_kinematics(q_zero, 'right')
                return current_fk[:3, :3]
        else:
            # For non-identity targets, apply the relative rotation to current natural orientation
            q_zero = np.zeros(22)
            if arm == 'left':
                current_fk = self.ik_solver.forward_kinematics(q_zero, 'left')
                relative_rotation = target_orientation_matrix @ identity.T
                return relative_rotation @ current_fk[:3, :3]
            else:
                current_fk = self.ik_solver.forward_kinematics(q_zero, 'right')
                relative_rotation = target_orientation_matrix @ identity.T
                return relative_rotation @ current_fk[:3, :3]
    
    def odom_callback(self, msg):
        """Callback for odometry messages to get current robot pose"""
        with self.odom_lock:
            # Extract position
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            # Extract orientation and convert to yaw angle
            quat = msg.pose.pose.orientation
            # Convert quaternion to euler angles to get yaw
            quat_array = [quat.w, quat.x, quat.y, quat.z]
            # Extract yaw from quaternion (rotation around z-axis)
            # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
            yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
            
            self.current_odom_pose = (x, y, yaw)
            self.get_logger().debug(f"Odom pose: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
    
    def target_poses_callback(self, msg):
        """Callback for target poses array (left arm first, right arm second)"""
        if len(msg.poses) >= 2:
            with self.target_lock:
                # Convert pose messages to transform matrices
                left_transform = self.pose_msg_to_transform(msg.poses[0])
                right_transform = self.pose_msg_to_transform(msg.poses[1])
                
                
                # Convert orientations from identity to natural orientations
                left_natural_orient = self.convert_target_orientation(left_transform[:3, :3], 'left')
                right_natural_orient = self.convert_target_orientation(right_transform[:3, :3], 'right')
                
                # Update target transforms with natural orientations
                # NOTE: may need to disable natural orient (comment out) when controlling robot with mocap data
                self.left_target = left_transform.copy()
                self.left_target[:3, :3] = left_natural_orient
                
                self.right_target = right_transform.copy()
                self.right_target[:3, :3] = right_natural_orient
                
                self.get_logger().debug(f"Received left target: ({msg.poses[0].position.x:.3f}, {msg.poses[0].position.y:.3f}, {msg.poses[0].position.z:.3f})")
                self.get_logger().debug(f"Received right target: ({msg.poses[1].position.x:.3f}, {msg.poses[1].position.y:.3f}, {msg.poses[1].position.z:.3f})")
        else:
            self.get_logger().warn(f"Expected 2 poses in target_poses, got {len(msg.poses)}")

    def _debug_timestamps(self):
        frame_ids = self.tf_buffer._getFrameStrings()
        if frame_ids:
            current_time = self.get_clock().now().nanoseconds * 1e-9
            timestamps = []
            for frame_id in frame_ids:
                for target_frame in frame_ids:
                    if frame_id != target_frame:
                        try:
                            transform = self.tf_buffer.lookup_transform(
                                target_frame, frame_id, rclpy.time.Time()
                            )
                            timestamp_sec = transform.header.stamp.sec
                            timestamp_nanosec = transform.header.stamp.nanosec
                            timestamp_float = timestamp_sec + timestamp_nanosec * 1e-9
                            age = current_time - timestamp_float
                            timestamps.append({
                                'source': frame_id,
                                'target': target_frame,
                                'timestamp': timestamp_float,
                                'age': age,
                                'sec': timestamp_sec,
                                'nanosec': timestamp_nanosec
                            })
                        except Exception as e:
                            continue
            
            # sort and summarize
            if timestamps:
                timestamps.sort(key=lambda x: x['timestamp'])
                print(f"\nCurrent ROS time: {current_time:.9f}")
                print(f"Found {len(timestamps)} transforms")
                print()
                for i, ts in enumerate(timestamps[:5]):
                    print(f"{ts['source']:>12} -> {ts['target']:<12} | "
                          f"Time: {ts['sec']}.{ts['nanosec']:09d} | "
                          f"Age: {ts['age']:6.3f}s")
                if len(timestamps) > 5:
                    print(f"... and {len(timestamps) - 5} more transforms")
                print()
                oldest = timestamps[0]['timestamp']
                newest = timestamps[-1]['timestamp']
                print(f"Oldest transform timestamp: {oldest:.9f} (age {current_time - oldest:.3f}s)")
                print(f"Newest transform timestamp: {newest:.9f} (age {current_time - newest:.3f}s)")
                print("="*60 + "\n")

    def pointcloud_callback(self, msg):
        """Callback for point cloud data - update SDF for obstacle avoidance"""
        try:
            # start_time = time.time()
            # Convert ROS PointCloud2 to numpy array
            points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)

            # time1 = time.time()
            
            # Filter out invalid points (NaN, inf)
            valid_mask = np.all(np.isfinite(points), axis=1)
            points = points[valid_mask]
            
            if len(points) == 0:
                self.get_logger().debug("No valid points in pointcloud")
                return
            
            # Transform points from camera_link frame to world frame
            try:
                # First, try to get the latest transform
                transform = self.tf_buffer.lookup_transform(
                    'world', 
                    msg.header.frame_id,
                    msg.header.stamp,
                    timeout=Duration(seconds=0.1)
                )
                
                # Extract transform components
                t = transform.transform.translation
                r = transform.transform.rotation
                
                # Convert quaternion to rotation matrix more efficiently
                from transforms3d import quaternions
                quat = [r.w, r.x, r.y, r.z]  # w, x, y, z order for transforms3d
                rotation_matrix = quaternions.quat2mat(quat).astype(np.float32)
                translation = np.array([t.x, t.y, t.z], dtype=np.float32)
                
                # Apply transformation directly without creating homogeneous coordinates
                # R @ points.T + t (broadcasted)
                points = (rotation_matrix @ points.T).T + translation
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Could not transform pointcloud from {msg.header.frame_id} to world: {e}")
                return
            except Exception as e:
                self.get_logger().error(f"Error during pointcloud transformation: {e}")
                return
            
            # time2 = time.time()
            
            # Apply filtering based on configuration (now in world frame)
            distances = np.linalg.norm(points, axis=1)
            range_mask = ((distances < MobileIKConfig.POINTCLOUD_MAX_RANGE) & 
                         (distances > MobileIKConfig.POINTCLOUD_MIN_RANGE))
            points = points[range_mask]
            
            # Downsample for efficiency
            if len(points) > MobileIKConfig.POINTCLOUD_DOWNSAMPLE_SIZE:
                indices = np.random.choice(len(points), MobileIKConfig.POINTCLOUD_DOWNSAMPLE_SIZE, replace=False)
                points = points[indices]

            # end_time = time.time()
            # self.get_logger().info(f"pc: convert={((time1 - start_time)*1000):.1f}ms transform={((time2 - time1)*1000):.1f}ms filter={((end_time - time2)*1000):.1f}ms total={((end_time - start_time)*1000):.1f}ms")
            
            with self.obstacle_lock:
                self.obstacle_points = points
                
                # Rate-limit SDF updates for performance
                self.sdf_update_counter += 1
                if self.sdf_update_counter >= MobileIKConfig.SDF_UPDATE_RATE:
                    self.sdf_update_counter = 0
                    
                    # Get current robot position for centering SDF bounds
                    robot_position = self._get_current_robot_position()
                    
                    # Update SDF with robot position for dynamic bounds
                    if len(points) > 0:
                        self.sdf.update_from_pointcloud(points, robot_position=robot_position)
                        self.get_logger().debug(f"Updated SDF with {len(points)} transformed obstacle points, centered at robot position: {robot_position}")
                    else:
                        # No obstacles detected
                        self.sdf.update_from_pointcloud(np.empty((0, 3)), robot_position=robot_position)
                        self.get_logger().debug(f"No obstacles detected after transformation, cleared SDF, centered at robot position: {robot_position}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def mode_callback(self, msg):
        """Callback for mode messages from /mode topic"""
        with self.mode_lock:
            self.current_mode = msg.data
        self.get_logger().debug(f"Mode updated to: {self.current_mode}")

    def frame_info_callback(self, msg):
        """Callback for frame info messages"""
        # Only log full message occasionally to avoid spam
        frame_idx = None
        for kv in msg.status[0].values:
            if kv.key == "frame_idx_loop":
                frame_idx = int(kv.value)   # convert to int if needed
                break

        if frame_idx is None:
            self.get_logger().warn("frame_idx_loop not found in message")
            return

        self.get_logger().debug(f"Current loop frame: {frame_idx} (previous: {self.frame_idx_loop})")
        
        # if the frame_idx_loop in msg.status[0].values drops (new loop), call the setpos service
        if frame_idx < self.frame_idx_loop:
            self.publish_pose = True
            self.frame_idx_loop = frame_idx
            self.get_logger().info(f"New loop detected: {frame_idx} < {self.frame_idx_loop}.")

    def set_pose(self, q):
        self.publish_pose = False

    def set_pose_response_callback(self, future):
        pass
    
    def publish_standby_sequence(self):
        """Publish 5 interpolated frames from zero position to standby target"""
        try:
            self.get_logger().info(f"Starting smooth standby sequence ({STANDBY_NUM_FRAMES} frames)...")
            
            init_position = np.zeros(len(self.standby_target))
            
            for i in range(STANDBY_NUM_FRAMES):
                # Linear interpolation from 0 to target
                alpha = (i + 1) / STANDBY_NUM_FRAMES
                interpolated_position = init_position + alpha * self.standby_target
                
                # Publish upper body commands
                cmd_msg = Float64MultiArray()
                cmd_msg.data = [float(pos) for pos in interpolated_position]
                self.upper_body_commands_pub.publish(cmd_msg)
                
                # Publish zero velocity for lower body (chassis)
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.linear.y = 0.0
                cmd_vel_msg.linear.z = 0.0
                cmd_vel_msg.angular.x = 0.0
                cmd_vel_msg.angular.y = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel_msg)
                
                self.get_logger().info(f"Published standby frame {i+1}/{STANDBY_NUM_FRAMES} (alpha={alpha:.2f})")
                
                # Wait for dt between frames
                time.sleep(STANDBY_FRAME_DELAY)
            
            # Update current_q with the final startup positions
            self.current_q[self.upper_joint_indices] = self.standby_target
            
            self.standby_complete = True

            self.get_logger().info("Standby sequence complete. Beginning normal control loop.")
            
        except Exception as e:
            self.get_logger().error(f"Error during standby sequence: {e}")
            self.standby_complete = True  # Continue anyway

    def publish_standby_sequence_ik(self):
        """Publish standby sequence for ia_robot_ik with virtual joints (no interpolation)"""
        try:
            self.get_logger().info("Publishing ia_robot_ik standby sequence...")
            
            # Target pose for lower body (virtual joints)
            # pos: [0.0, -1.7, 0.0], euler: [0.0, 0.0, 90.0]
            target_x = 0.0
            target_y = -1.7
            target_yaw = np.radians(90.0)  # Convert 90 degrees to radians
            
            # Target positions for upper body (same as standby_target)
            upper_body_positions = self.standby_target
            
            # Create JointState message with all joints
            joint_msg = JointState()
            joint_msg.header = Header()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = 'world'
            
            # Lower body (virtual joints)
            joint_msg.name = self.chassis_joint_names + self.upper_joint_names
            joint_msg.position = [
                float(target_x),      # chassis_x_joint
                float(target_y),      # chassis_y_joint
                float(target_yaw)     # chassis_rotation_joint
            ] + [float(pos) for pos in upper_body_positions]
            
            joint_msg.velocity = [0.0] * len(joint_msg.name)
            joint_msg.effort = [0.0] * len(joint_msg.name)
            
            # Publish joint states
            self.joint_states_pub.publish(joint_msg)
            
            self.get_logger().info(f"Published ia_robot_ik standby: lower_body=[{target_x}, {target_y}, {target_yaw:.3f}], upper_body={len(upper_body_positions)} joints")
            
            self.standby_complete = True
            self.get_logger().info("ia_robot_ik standby sequence complete.")
            
        except Exception as e:
            self.get_logger().error(f"Error during ia_robot_ik standby sequence: {e}")
            self.standby_complete = True  # Continue anyway
    
    ################ MoveToStandby Action #######################
    def goal_callback(self, goal_request):
        """Handle incoming action goal requests"""
        self.get_logger().info('Received move to standby goal request')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle action cancellation requests"""
        self.get_logger().info('Received cancel request for move to standby')
        return CancelResponse.ACCEPT
    
    def execute_standby_callback(self, goal_handle):
        """
        Execute move to standby action.
        This sets a flag that the control_loop() will respect.
        The actual command publishing happens in control_loop() to avoid conflicts.
        This callback blocks but runs in a separate thread thanks to ReentrantCallbackGroup.
        """
        self.get_logger().info('Executing move to standby action...')
        
        try:
            # Check if joints are locked - cannot move to standby
            if self.joints_locked:
                self.get_logger().warn('Cannot move to standby: joints are locked')
                result = MoveToStandby.Result()
                result.success = False
                result.message = 'Cannot move to standby: joints are locked'
                goal_handle.abort()
                return result
            
            # Set action active flag - control_loop will handle the actual movement
            self.standby_action_active = True
            self.standby_action_progress = 0
            self.standby_action_goal_handle = goal_handle
            
            # Store initial upper body positions when action starts
            self.standby_initial_position = self.current_q[self.upper_joint_indices].copy()
            
            # Wait for action to complete (control_loop will update progress)
            # This blocks but it's OK because we're using ReentrantCallbackGroup
            while self.standby_action_active and rclpy.ok():
                # Check if canceled
                if goal_handle.is_cancel_requested:
                    self.standby_action_active = False
                    goal_handle.canceled()
                    result = MoveToStandby.Result()
                    result.success = False
                    result.message = 'Move to standby canceled'
                    return result
                
                # Send feedback
                if self.standby_action_progress > 0:
                    feedback_msg = MoveToStandby.Feedback()
                    feedback_msg.current_frame = self.standby_action_progress
                    feedback_msg.total_frames = STANDBY_NUM_FRAMES
                    feedback_msg.progress_percentage = (self.standby_action_progress / STANDBY_NUM_FRAMES) * 100.0
                    goal_handle.publish_feedback(feedback_msg)
                
                # Sleep briefly to avoid busy waiting
                time.sleep(0.05)
            
            # Check if completed successfully or failed
            if self.standby_action_progress >= STANDBY_NUM_FRAMES:
                goal_handle.succeed()
                result = MoveToStandby.Result()
                result.success = True
                result.message = 'Successfully moved to standby position'
                self.get_logger().info('Move to standby action completed successfully')
                return result
            else:
                goal_handle.abort()
                result = MoveToStandby.Result()
                result.success = False
                result.message = 'Action aborted or interrupted'
                return result
            
        except Exception as e:
            self.get_logger().error(f"Move to standby action failed: {e}")
            self.standby_action_active = False
            goal_handle.abort()
            result = MoveToStandby.Result()
            result.success = False
            result.message = f'Action failed: {str(e)}'
            return result
    
    def execute_standby_in_control_loop(self):
        """
        Execute standby movement within the control loop.
        This is called by control_loop() when standby_action_active is True.
        Runs at the same rate as control_loop (20Hz).
        """
        try:
            # Calculate which frame we should be at based on time
            # Since control_loop runs at 20Hz (0.05s) and STANDBY_FRAME_DELAY is 0.2s,
            # we need 4 control loop iterations per standby frame
            iterations_per_frame = int(STANDBY_FRAME_DELAY / self.dt)
            
            # Track iteration count (initialize if needed)
            if not hasattr(self, 'standby_iteration_count'):
                self.standby_iteration_count = 0
            
            self.standby_iteration_count += 1
            
            # Calculate current frame
            current_frame = min(self.standby_iteration_count // iterations_per_frame + 1, STANDBY_NUM_FRAMES)
            
            # Update progress
            self.standby_action_progress = current_frame
            
            # Calculate interpolation
            alpha = current_frame / STANDBY_NUM_FRAMES
            
            # Use stored initial position (captured when action started)
            if not hasattr(self, 'standby_initial_position'):
                # Fallback to zeros if not set (shouldn't happen)
                init_position = np.zeros(len(self.standby_target))
                self.get_logger().warn("Using zero init_position")
            else:
                init_position = self.standby_initial_position
            
            interpolated_position = init_position + alpha * (self.standby_target - init_position)
            
            # Publish upper body commands
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [float(pos) for pos in interpolated_position]
            self.upper_body_commands_pub.publish(cmd_msg)
            
            # Publish zero velocity for chassis
            cmd_vel_msg = Twist()
            self.cmd_vel_pub.publish(cmd_vel_msg)
            
            # Log progress at frame boundaries
            if self.standby_iteration_count % iterations_per_frame == 0:
                self.get_logger().info(
                    f"Standby frame {current_frame}/{STANDBY_NUM_FRAMES} ({alpha*100:.1f}%)"
                )
            
            # Check if completed
            if current_frame >= STANDBY_NUM_FRAMES:
                self.get_logger().info('Standby action completed in control loop')
                # Update current_q with the final standby positions
                self.current_q[self.upper_joint_indices] = self.standby_target
                self.standby_action_active = False
                self.standby_iteration_count = 0
                
        except Exception as e:
            self.get_logger().error(f"Error executing standby in control loop: {e}")
            self.standby_action_active = False
            self.standby_iteration_count = 0
    ################ MoveToStandby Action END ####################

    ################ JointLock Service #######################
    def handle_joint_lock(self, request, response):
        """Handle joint lock service request"""
        self.joints_locked = request.lock
        response.success = True
        response.message = f"Joints {'locked' if request.lock else 'unlocked'}"
        self.get_logger().info(response.message)
        return response
    ################ JointLock Service END ####################

    def pose_msg_to_transform(self, pose):
        """Convert ROS Pose message to 4x4 transformation matrix"""
        # Extract position
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        
        # Extract orientation and convert to rotation matrix
        quat = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]  # w,x,y,z order
        rotation_matrix = quaternions.quat2mat(quat)
        
        # Create homogeneous transformation matrix
        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = position
        
        return transform
    
    def control_loop(self):
        """Main control loop"""
        # Skip control loop until standby sequence is complete
        if not self.standby_complete:
            return
        
        # Send zero velocity if joints are locked
        if self.joints_locked:
            # Publish zero velocity for chassis
            cmd_vel_msg = Twist()
            self.cmd_vel_pub.publish(cmd_vel_msg)
            return
        
        # Handle standby action if active
        if self.standby_action_active:
            self.execute_standby_in_control_loop()
            return
        
        # Publish current ARM2 poses from TF
        self.publish_current_arm2_poses()
        
        with self.target_lock:
            if self.left_target is None or self.right_target is None:
                return  # Wait for both targets
            
            left_target = self.left_target.copy()
            right_target = self.right_target.copy()
        
        # Get current mode
        with self.mode_lock:
            current_mode = self.current_mode
        
        try:
            # Solve IK
            q_solution, success = self.ik_solver.inverse_kinematics(
                left_target, 
                right_target, 
                self.current_q,
                mode=current_mode
            )
            
            if not success:
                self.get_logger().warn("IK solver failed to converge")
            
            # Update current configuration
            self.current_q = q_solution.copy()


            # Publish all joint states (including chassis joints) in world frame
            self.publish_joint_states(q_solution)

            if self.publish_pose:
                self.set_pose(q_solution)

            # Publish upper body commands
            self.publish_upper_body_commands(q_solution)

            # Publish chassis commands
            self.publish_chassis_commands(q_solution)

            # Publish FK results for debugging
            self.publish_fk_results(q_solution)
            
            # Publish collision avoidance visualization
            self.publish_collision_visualization(q_solution)
            
            # self._debug_timestamps()
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")
    
    def publish_current_arm2_poses(self):
        """Get ARM2_LEFT and ARM2_RIGHT poses from TF and publish as PoseArray"""
        try:
            # Get current time
            now = rclpy.time.Time()
            
            # Lookup transforms for ARM2_LEFT and ARM2_RIGHT in odom frame
            try:
                transform_left = self.tf_buffer.lookup_transform(
                    'odom',
                    'ARM2_LEFT',
                    now,
                    timeout=Duration(seconds=0.1)
                )
                
                transform_right = self.tf_buffer.lookup_transform(
                    'odom',
                    'ARM2_RIGHT',
                    now,
                    timeout=Duration(seconds=0.1)
                )
                
                # Create PoseArray message
                pose_array = PoseArray()
                pose_array.header = Header()
                pose_array.header.stamp = self.get_clock().now().to_msg()
                pose_array.header.frame_id = 'odom'
                
                # Apply end effector offsets (same as in Pinocchio model)
                # Left: [-0.15, 0, 0.05], Right: [-0.15, 0, -0.05]
                left_ee_offset = np.array([-0.15, 0, 0.05])
                right_ee_offset = np.array([-0.15, 0, -0.05])
                
                # Convert transform to numpy for easier manipulation
                # Extract rotation matrix from quaternion for left arm
                quat_left = [
                    transform_left.transform.rotation.w,
                    transform_left.transform.rotation.x,
                    transform_left.transform.rotation.y,
                    transform_left.transform.rotation.z
                ]
                rot_matrix_left = quaternions.quat2mat(quat_left)
                
                # Apply offset in local frame
                pos_left = np.array([
                    transform_left.transform.translation.x,
                    transform_left.transform.translation.y,
                    transform_left.transform.translation.z
                ])
                pos_left_transformed = pos_left + rot_matrix_left @ left_ee_offset
                
                # Extract rotation matrix from quaternion for right arm
                quat_right = [
                    transform_right.transform.rotation.w,
                    transform_right.transform.rotation.x,
                    transform_right.transform.rotation.y,
                    transform_right.transform.rotation.z
                ]
                rot_matrix_right = quaternions.quat2mat(quat_right)
                
                # Apply offset in local frame
                pos_right = np.array([
                    transform_right.transform.translation.x,
                    transform_right.transform.translation.y,
                    transform_right.transform.translation.z
                ])
                pos_right_transformed = pos_right + rot_matrix_right @ right_ee_offset
                
                # Convert transforms to Pose messages with offsets applied
                left_pose = Pose()
                left_pose.position.x = float(pos_left_transformed[0])
                left_pose.position.y = float(pos_left_transformed[1])
                left_pose.position.z = float(pos_left_transformed[2])
                left_pose.orientation = transform_left.transform.rotation
                
                right_pose = Pose()
                right_pose.position.x = float(pos_right_transformed[0])
                right_pose.position.y = float(pos_right_transformed[1])
                right_pose.position.z = float(pos_right_transformed[2])
                right_pose.orientation = transform_right.transform.rotation
                
                # Keep original order (no swap), and reverse orientation (180 degrees around Z)
                reversed_poses = []
                for pose in [left_pose, right_pose]:  # Original order
                    # Original quaternion
                    q = pose.orientation
                    x1, y1, z1, w1 = q.x, q.y, q.z, q.w

                    # 180 degree rotation around Z: [0, 0, 1, 0]
                    x2, y2, z2, w2 = 0.0, 0.0, 1.0, 0.0

                    # Quaternion multiplication: q_original * q_180z
                    reversed_pose = Pose()
                    reversed_pose.position = pose.position
                    reversed_pose.orientation = Quaternion(
                        x=float(w1*x2 + x1*w2 + y1*z2 - z1*y2),
                        y=float(w1*y2 - x1*z2 + y1*w2 + z1*x2),
                        z=float(w1*z2 + x1*y2 - y1*x2 + z1*w2),
                        w=float(w1*w2 - x1*x2 - y1*y2 - z1*z2)
                    )
                    reversed_poses.append(reversed_pose)

                pose_array.poses = reversed_poses
                
                # Publish
                self.current_poses_pub.publish(pose_array)
                
                self.get_logger().debug(f"Published ARM2 poses - Left: ({left_pose.position.x:.3f}, {left_pose.position.y:.3f}, {left_pose.position.z:.3f}), Right: ({right_pose.position.x:.3f}, {right_pose.position.y:.3f}, {right_pose.position.z:.3f})")
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().debug(f"Could not get ARM2 transforms: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing ARM2 poses: {e}")
    
    def publish_chassis_commands(self, q):
        """Extract chassis motion and publish as cmd_vel with noise filtering"""
        try:
            # Get current pose from odometry
            with self.odom_lock:
                if self.current_odom_pose is None:
                    self.get_logger().warn("No odometry data received yet")
                    return
                current_x, current_y, current_theta = self.current_odom_pose
            
            # Get target chassis pose from IK solution
            target_x, target_y, target_theta = self.ik_solver.get_chassis_pose(q)

            # Calculate chassis velocities using target and current poses
            # Linear velocities in world frame
            dx_world = target_x - current_x
            dy_world = target_y - current_y
            dtheta = target_theta - current_theta
            
            # Handle angle wrapping
            if dtheta > np.pi:
                dtheta -= 2 * np.pi
            elif dtheta < -np.pi:
                dtheta += 2 * np.pi
            
            # Transform world velocities to chassis frame using current orientation
            cos_theta = np.cos(current_theta)
            sin_theta = np.sin(current_theta)
            
            vx_chassis = cos_theta * dx_world + sin_theta * dy_world
            vy_chassis = -sin_theta * dx_world + cos_theta * dy_world
            
            # Calculate raw velocities
            raw_vx = vx_chassis / self.dt
            raw_vy = vy_chassis / self.dt
            raw_wz = dtheta / self.dt
            
            # Apply moving average filter
            filtered_vx = self.apply_moving_average_filter('vx', raw_vx)
            filtered_vy = self.apply_moving_average_filter('vy', raw_vy)
            filtered_wz = self.apply_moving_average_filter('wz', raw_wz)
            
            # Apply velocity limits
            max_linear_vel = 0.5  # m/s
            max_angular_vel = 1.0  # rad/s
            
            filtered_vx = np.clip(filtered_vx, -max_linear_vel, max_linear_vel)
            filtered_vy = np.clip(filtered_vy, -max_linear_vel, max_linear_vel)
            filtered_wz = np.clip(filtered_wz, -max_angular_vel, max_angular_vel)
            
            # Apply threshold filter (deadzone for small velocities)
            if abs(filtered_vx) < self.velocity_threshold:
                filtered_vx = 0.0
            if abs(filtered_vy) < self.velocity_threshold:
                filtered_vy = 0.0
            if abs(filtered_wz) < self.velocity_threshold:
                filtered_wz = 0.0
            
            # Create and publish Twist message
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = filtered_vx
            cmd_vel_msg.linear.y = filtered_vy
            cmd_vel_msg.linear.z = 0.0
            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = filtered_wz
            
            self.cmd_vel_pub.publish(cmd_vel_msg)
            
            # Log chassis motion
            if abs(cmd_vel_msg.linear.x) > 0.01 or abs(cmd_vel_msg.linear.y) > 0.01 or abs(cmd_vel_msg.angular.z) > 0.01:
                self.get_logger().debug(f"Chassis cmd_vel: vx={cmd_vel_msg.linear.x:.3f}, vy={cmd_vel_msg.linear.y:.3f}, wz={cmd_vel_msg.angular.z:.3f}")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing chassis commands: {e}")
    
    def apply_moving_average_filter(self, vel_type, raw_velocity):
        """Apply moving average filter to reduce noise"""
        # Add new velocity to history
        self.velocity_history[vel_type].append(raw_velocity)
        
        # Keep only recent history
        if len(self.velocity_history[vel_type]) > self.history_length:
            self.velocity_history[vel_type].pop(0)
        
        # Return moving average
        return np.mean(self.velocity_history[vel_type])
    
    def publish_joint_states(self, q):
        """Publish all joint positions as joint_states in world frame"""
        try:
            # Create JointState message
            joint_msg = JointState()
            joint_msg.header = Header()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = 'world'  # All joints in world coordinate frame
            
            # Publish all joints (including chassis joints)
            joint_names = []
            joint_positions = []
            joint_velocities = []
            
            for i, name in enumerate(self.joint_names):
                joint_names.append(name)
                joint_positions.append(float(q[i]))
                
                # Calculate velocity if we have previous state
                if self.previous_q is not None:
                    vel = (q[i] - self.previous_q[i]) / self.dt
                    joint_velocities.append(float(vel))
                else:
                    joint_velocities.append(0.0)
            
            joint_msg.name = joint_names
            joint_msg.position = joint_positions
            joint_msg.velocity = joint_velocities
            joint_msg.effort = [0.0] * len(joint_names)  # No effort feedback for now
            
            self.joint_states_pub.publish(joint_msg)
            
            # Update previous joint state
            self.previous_q = q.copy()
            
            self.get_logger().debug(f"Published {len(joint_names)} joint states in world frame")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing joint states: {e}")

    def publish_upper_body_commands(self, q):
        """Publish upper body joint commands"""
        try:
            # Create Float64MultiArray message
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [float(q[j]) for j in self.upper_joint_indices]
            self.upper_body_commands_pub.publish(cmd_msg)

            self.get_logger().debug(f"Published upper body commands: {cmd_msg.data}")

        except Exception as e:
            self.get_logger().error(f"Error publishing upper body commands: {e}")

    def publish_fk_results(self, q):
        """Publish FK results based on current robot joint positions"""
        try:
            # Compute FK based on current robot joint positions (original, no modifications)
            left_fk = self.ik_solver.forward_kinematics(self.current_q, 'left')
            right_fk = self.ik_solver.forward_kinematics(self.current_q, 'right')
            
            # Create PoseArray message
            fk_msg = PoseArray()
            fk_msg.header = Header()
            fk_msg.header.stamp = self.get_clock().now().to_msg()
            fk_msg.header.frame_id = 'odom'
            
            # Convert corrected FK results to Pose messages
            left_pose = self.transform_to_pose_msg(left_fk)
            right_pose = self.transform_to_pose_msg(right_fk)
            
            # Publish only current robot EEF poses [left_eef, right_eef]
            fk_msg.poses = [left_pose, right_pose]
            
            self.fk_result_pub.publish(fk_msg)
            
            # Optional: Log current EEF positions for debugging
            self.get_logger().debug(f"Current EEF positions - Left: ({left_fk[:3, 3][0]:.3f}, {left_fk[:3, 3][1]:.3f}, {left_fk[:3, 3][2]:.3f}), Right: ({right_fk[:3, 3][0]:.3f}, {right_fk[:3, 3][1]:.3f}, {right_fk[:3, 3][2]:.3f})")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing FK results: {e}")
    
    def transform_to_pose_msg(self, transform):
        """Convert 4x4 transformation matrix to Pose message"""
        pose = Pose()
        
        # Position
        pose.position.x = float(transform[0, 3])
        pose.position.y = float(transform[1, 3])
        pose.position.z = float(transform[2, 3])
        
        # Orientation (convert rotation matrix to quaternion)
        rot_matrix = transform[:3, :3]
        quat = quaternions.mat2quat(rot_matrix)  # returns [w, x, y, z]
        pose.orientation.w = float(quat[0])
        pose.orientation.x = float(quat[1])
        pose.orientation.y = float(quat[2])
        pose.orientation.z = float(quat[3])
        
        return pose
    
    def publish_collision_visualization(self, q):
        """Publish visualization data for collision avoidance"""
        try:
            if self.collision_checker is not None and self.sdf.is_valid:
                # Compute collision distances
                distances, total_cost = self.collision_checker.compute_collision_distances(q)
                
                # Publish collision distances
                distances_msg = Float32MultiArray()
                distances_msg.data = [float(d) for d in distances]
                self.collision_distances_pub.publish(distances_msg)
                
                # Publish processed obstacle points
                with self.obstacle_lock:
                    if self.obstacle_points is not None and len(self.obstacle_points) > 0:
                        obstacle_msg = PointCloud2()
                        obstacle_msg.header = Header()
                        obstacle_msg.header.stamp = self.get_clock().now().to_msg()
                        obstacle_msg.header.frame_id = 'world'
                        
                        # Convert numpy array to PointCloud2
                        obstacle_msg = point_cloud2.create_cloud_xyz32(obstacle_msg.header, self.obstacle_points)
                        self.obstacle_points_pub.publish(obstacle_msg)
                
                # Publish SDF visualization (sample some points for visualization)
                self.publish_sdf_markers()
                
                self.get_logger().debug(f"Collision distances: {distances}, Total cost: {total_cost:.3f}")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing collision visualization: {e}")
    
    def _get_current_robot_position(self):
        """
        Get current robot position in world coordinates for centering SDF bounds.
        Returns the chassis position as the robot center.
        """
        try:
            # Get chassis position from current joint configuration
            chassis_x = self.current_q[0] if len(self.current_q) > 0 else 0.0
            chassis_y = self.current_q[1] if len(self.current_q) > 1 else 0.0
            chassis_z = 0.6  # Assume robot base is at 0.6m height
            
            return np.array([chassis_x, chassis_y, chassis_z])
        except Exception as e:
            self.get_logger().warn(f"Could not get robot position: {e}, using origin")
            return np.array([0.0, 0.0, 0.6])
    
    def publish_sdf_markers(self):
        """Publish SDF visualization as markers"""
        try:
            if not self.sdf.is_valid:
                return
            
            marker_array = MarkerArray()
            
            # Sample SDF grid for visualization (downsample for performance)
            sample_step = max(1, self.sdf.grid_size[0] // MobileIKConfig.SDF_VISUALIZATION_SAMPLE_RATE)
            
            marker_id = 0
            for i in range(0, self.sdf.grid_size[0], sample_step):
                for j in range(0, self.sdf.grid_size[1], sample_step):
                    for k in range(0, self.sdf.grid_size[2], sample_step):
                        distance = self.sdf.sdf[i, j, k]
                        
                        # Only visualize points close to obstacles
                        if distance < MobileIKConfig.SDF_VISUALIZATION_THRESHOLD:  
                            marker = Marker()
                            marker.header.frame_id = 'world'
                            marker.header.stamp = self.get_clock().now().to_msg()
                            marker.ns = 'sdf'
                            marker.id = marker_id
                            marker.type = Marker.SPHERE
                            marker.action = Marker.ADD
                            
                            # Convert grid indices to world coordinates
                            world_pos = self.sdf.grid_to_world(np.array([i, j, k]))
                            marker.pose.position.x = float(world_pos[0])
                            marker.pose.position.y = float(world_pos[1])
                            marker.pose.position.z = float(world_pos[2])
                            marker.pose.orientation.w = 1.0
                            
                            # Scale and color based on distance
                            marker.scale.x = marker.scale.y = marker.scale.z = 0.05
                            
                            # Color: red for close, yellow for medium distance
                            if distance < 0.1:
                                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
                            elif distance < 0.3:
                                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
                            else:
                                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
                            
                            marker.color.a = 0.6
                            marker.lifetime.sec = 1  # 1 second lifetime
                            
                            marker_array.markers.append(marker)
                            marker_id += 1
                            
                            # Limit number of markers for performance
                            if marker_id > MobileIKConfig.VISUALIZATION_MAX_MARKERS:
                                break
            
            self.sdf_markers_pub.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing SDF markers: {e}")

def main(args=None):
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Mobile IK Controller with Obstacle Avoidance')
    parser.add_argument('--use-ik', action='store_true', 
                        help='Use IK startup sequence instead of standard startup sequence')
    parsed_args, ros_args = parser.parse_known_args()
    
    rclpy.init(args=ros_args)
    
    try:
        controller = MobileIKController(use_ik=parsed_args.use_ik)
        
        print("Mobile IK Controller with Obstacle Avoidance started.")
        print(f"Startup mode: {'IK sequence' if parsed_args.use_ik else 'Standard sequence'}")
        print("Subscribed to:")
        print("  - /demo_target_poses (geometry_msgs/PoseArray) - expects 2 poses: [left_arm, right_arm]")
        print("  - /points_dep (sensor_msgs/PointCloud2) - obstacle detection")
        print("")
        print("Publishing:")
        print("  - All joint states (including chassis) to /joint_states (sensor_msgs/JointState) in world frame")
        print("  - FK results for debugging to /fk_results (geometry_msgs/PoseArray)")
        print("  - Collision distances to /collision_distances (std_msgs/Float32MultiArray)")
        print("  - SDF visualization to /sdf_visualization (visualization_msgs/MarkerArray)")
        print("  - Processed obstacles to /processed_obstacles (sensor_msgs/PointCloud2)")
        print("")
        print("Features:")
        print("  - Signed Distance Field (SDF) for efficient obstacle representation")
        print("  - Real-time collision checking with robot links") 
        print("  - Obstacle avoidance integrated into IK optimization")
        print("")
        print("Usage:")
        print("  python3 kinematics/mobile_ik_controller.py              # Use standard startup sequence")
        print("  python3 kinematics/mobile_ik_controller.py --use-ik     # Use IK startup sequence")
        print("")
        print("Press Ctrl+C to stop...")
        
        # Use MultiThreadedExecutor to allow action callbacks to run concurrently with timers
        executor = MultiThreadedExecutor()
        executor.add_node(controller)
        executor.spin()
        
    except KeyboardInterrupt:
        print("\nController stopped by user")
    except Exception as e:
        print(f"Controller failed: {e}")
    finally:
        if 'controller' in locals():
            # Cleanup TF thread before destroying node
            if hasattr(controller, 'tf_thread_running'):
                controller.tf_thread_running = False
            if hasattr(controller, 'tf_thread') and controller.tf_thread.is_alive():
                controller.tf_thread.join(timeout=2.0)
            if hasattr(controller, 'tf_node'):
                controller.tf_node.destroy_node()
            controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()