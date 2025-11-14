#!/usr/bin/env python3
"""
实时可视化机器人位置、目标位置、控制指令和方向
订阅相关ROS2话题并使用matplotlib实时绘图
所有坐标统一变换到map坐标系
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrow, Rectangle
from matplotlib.animation import FuncAnimation
import threading
import tf2_ros
from rclpy.duration import Duration


class MobileIKVisualizer(Node):
    def __init__(self):
        super().__init__('mobile_ik_visualizer')

        # Data storage (all in map frame)
        self.robot_pose = None  # (x, y, yaw) in map frame
        self.left_target = None  # (x, y, z) in map frame
        self.right_target = None  # (x, y, z) in map frame
        self.left_current = None  # (x, y, z) in map frame
        self.right_current = None  # (x, y, z) in map frame
        self.cmd_vel = None  # (vx, vy, wz) in base_link frame

        # History for trajectories
        self.max_history = 100
        self.robot_history = []
        self.left_target_history = []
        self.right_target_history = []

        # Thread lock
        self.data_lock = threading.Lock()

        # TF2 buffer and listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.target_poses_sub = self.create_subscription(
            PoseArray,
            '/demo_target_poses',
            self.target_poses_callback,
            10
        )

        self.current_poses_sub = self.create_subscription(
            PoseArray,
            '/current_poses',
            self.current_poses_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('Mobile IK Visualizer initialized')
        self.get_logger().info('Waiting for data...')

    def odom_callback(self, msg):
        """接收机器人位姿数据，变换到map坐标系"""
        try:
            # 通过TF获取从base_footprint到map的变换
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )

            with self.data_lock:
                # 从变换中提取位置
                x = transform.transform.translation.x
                y = transform.transform.translation.y

                # 提取yaw角度
                quat = transform.transform.rotation
                yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                               1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))

                self.robot_pose = (x, y, yaw)

                # 添加到历史记录
                self.robot_history.append((x, y))
                if len(self.robot_history) > self.max_history:
                    self.robot_history.pop(0)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f"TF lookup failed for robot pose: {e}")

    def target_poses_callback(self, msg):
        """接收目标位姿数据 [left_arm, right_arm]，已经在map坐标系"""
        # /demo_target_poses 已经在 map 坐标系，直接使用
        with self.data_lock:
            if len(msg.poses) >= 2:
                left = msg.poses[0].position
                right = msg.poses[1].position
                self.left_target = (left.x, left.y, left.z)
                self.right_target = (right.x, right.y, right.z)

                # 添加到历史记录
                self.left_target_history.append((left.x, left.y))
                if len(self.left_target_history) > self.max_history:
                    self.left_target_history.pop(0)

                self.right_target_history.append((right.x, right.y))
                if len(self.right_target_history) > self.max_history:
                    self.right_target_history.pop(0)

    def current_poses_callback(self, msg):
        """接收当前手臂位姿数据 [left_arm, right_arm]，从odom变换到map"""
        # /current_poses 在 odom 坐标系，需要变换到 map
        try:
            # 获取 odom 到 map 的变换
            transform = self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )

            # 提取变换矩阵
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            quat = transform.transform.rotation
            # 简化处理：只考虑Z轴旋转（2D平面）
            yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                           1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)

            with self.data_lock:
                if len(msg.poses) >= 2:
                    # 左臂变换
                    left = msg.poses[0].position
                    left_x_map = tx + cos_yaw * left.x - sin_yaw * left.y
                    left_y_map = ty + sin_yaw * left.x + cos_yaw * left.y
                    left_z_map = tz + left.z
                    self.left_current = (left_x_map, left_y_map, left_z_map)

                    # 右臂变换
                    right = msg.poses[1].position
                    right_x_map = tx + cos_yaw * right.x - sin_yaw * right.y
                    right_y_map = ty + sin_yaw * right.x + cos_yaw * right.y
                    right_z_map = tz + right.z
                    self.right_current = (right_x_map, right_y_map, right_z_map)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f"TF lookup failed for current poses: {e}")

    def cmd_vel_callback(self, msg):
        """接收控制速度指令"""
        with self.data_lock:
            self.cmd_vel = (msg.linear.x, msg.linear.y, msg.angular.z)

    def get_data_snapshot(self):
        """获取数据快照（线程安全）"""
        with self.data_lock:
            return {
                'robot_pose': self.robot_pose,
                'left_target': self.left_target,
                'right_target': self.right_target,
                'left_current': self.left_current,
                'right_current': self.right_current,
                'cmd_vel': self.cmd_vel,
                'robot_history': self.robot_history.copy(),
                'left_target_history': self.left_target_history.copy(),
                'right_target_history': self.right_target_history.copy(),
            }


def spin_ros(node, stop_event):
    """ROS2 spin函数，在单独线程中运行"""
    while not stop_event.is_set() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    rclpy.init()

    # 创建节点
    visualizer = MobileIKVisualizer()

    # 创建停止事件
    stop_event = threading.Event()

    # 启动ROS2 spin线程
    ros_thread = threading.Thread(target=spin_ros, args=(visualizer, stop_event), daemon=True)
    ros_thread.start()

    # 创建matplotlib图形
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle('Mobile IK Controller Visualization (Map Frame)', fontsize=16, fontweight='bold')

    # 子图1: 顶视图 - 机器人和目标位置
    ax1.set_title('Top View: Robot & Target Positions (Map Frame)')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # 子图2: 侧视图 - Z轴高度
    ax2.set_title('Side View: Z Height (Map Frame)')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Z (m)')
    ax2.grid(True, alpha=0.3)

    # 子图3: 控制指令
    ax3.set_title('Control Commands (cmd_vel)')
    ax3.set_xlabel('Time Step')
    ax3.set_ylabel('Velocity')
    ax3.grid(True, alpha=0.3)

    # 子图4: 误差分析
    ax4.set_title('Position Error')
    ax4.set_xlabel('Time Step')
    ax4.set_ylabel('Error (m)')
    ax4.grid(True, alpha=0.3)

    # 历史数据记录
    cmd_vel_history = {'vx': [], 'vy': [], 'wz': []}
    error_history = {'left': [], 'right': []}
    max_cmd_history = 100

    def update_plot(frame):
        """更新图形的回调函数"""
        data = visualizer.get_data_snapshot()

        # 清空所有子图
        ax1.clear()
        ax2.clear()
        ax3.clear()
        ax4.clear()

        # ========== 子图1: 顶视图 ==========
        ax1.set_title('Top View: Robot & Target Positions (Map Frame)')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)

        # 绘制机器人
        if data['robot_pose']:
            x, y, yaw = data['robot_pose']

            # 机器人本体（矩形）
            robot_width = 0.6
            robot_length = 1.1
            robot_rect = Rectangle(
                (x - robot_length/2, y - robot_width/2),
                robot_length, robot_width,
                angle=np.degrees(yaw),
                rotation_point='center',
                fill=True, facecolor='lightblue', edgecolor='blue', linewidth=2
            )
            ax1.add_patch(robot_rect)

            # 机器人方向箭头
            arrow_length = 0.5
            dx = arrow_length * np.cos(yaw)
            dy = arrow_length * np.sin(yaw)
            ax1.arrow(x, y, dx, dy, head_width=0.15, head_length=0.1,
                     fc='red', ec='red', linewidth=2, label='Robot Direction')

            # 机器人位置点
            ax1.plot(x, y, 'bo', markersize=10, label='Robot Base')

            # 绘制位置死区（1cm半径的圆）
            deadzone_circle = Circle((x, y), 0.01, fill=False,
                                    edgecolor='orange', linestyle='--',
                                    linewidth=1, alpha=0.5, label='Position Deadzone (1cm)')
            ax1.add_patch(deadzone_circle)

        # 绘制机器人轨迹
        if data['robot_history']:
            hist = np.array(data['robot_history'])
            ax1.plot(hist[:, 0], hist[:, 1], 'b--', alpha=0.5, linewidth=1, label='Robot Path')

        # 绘制左臂目标和当前位置
        if data['left_target']:
            ax1.plot(data['left_target'][0], data['left_target'][1], 'r*',
                    markersize=15, label='Left Target')
        if data['left_current']:
            ax1.plot(data['left_current'][0], data['left_current'][1], 'ro',
                    markersize=10, label='Left Current')

        # 绘制左臂轨迹
        if data['left_target_history']:
            hist = np.array(data['left_target_history'])
            ax1.plot(hist[:, 0], hist[:, 1], 'r--', alpha=0.3, linewidth=1)

        # 绘制右臂目标和当前位置
        if data['right_target']:
            ax1.plot(data['right_target'][0], data['right_target'][1], 'g*',
                    markersize=15, label='Right Target')
        if data['right_current']:
            ax1.plot(data['right_current'][0], data['right_current'][1], 'go',
                    markersize=10, label='Right Current')

        # 绘制右臂轨迹
        if data['right_target_history']:
            hist = np.array(data['right_target_history'])
            ax1.plot(hist[:, 0], hist[:, 1], 'g--', alpha=0.3, linewidth=1)

        # 绘制连线（当前位置到目标）
        if data['left_target'] and data['left_current']:
            ax1.plot([data['left_current'][0], data['left_target'][0]],
                    [data['left_current'][1], data['left_target'][1]],
                    'r:', linewidth=1, alpha=0.5)

        if data['right_target'] and data['right_current']:
            ax1.plot([data['right_current'][0], data['right_target'][0]],
                    [data['right_current'][1], data['right_target'][1]],
                    'g:', linewidth=1, alpha=0.5)

        ax1.legend(loc='upper right', fontsize=8)

        # ========== 子图2: 侧视图（Z轴高度）==========
        ax2.set_title('Side View: Z Height (Map Frame)')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Z (m)')
        ax2.grid(True, alpha=0.3)

        if data['left_target']:
            ax2.plot(data['left_target'][0], data['left_target'][2], 'r*',
                    markersize=15, label='Left Target')
        if data['left_current']:
            ax2.plot(data['left_current'][0], data['left_current'][2], 'ro',
                    markersize=10, label='Left Current')

        if data['right_target']:
            ax2.plot(data['right_target'][0], data['right_target'][2], 'g*',
                    markersize=15, label='Right Target')
        if data['right_current']:
            ax2.plot(data['right_current'][0], data['right_current'][2], 'go',
                    markersize=10, label='Right Current')

        # 机器人底座高度
        if data['robot_pose']:
            ax2.axhline(y=0.0, color='blue', linestyle='--', alpha=0.5, label='Robot Base')

        ax2.legend(loc='upper right', fontsize=8)

        # ========== 子图3: 控制指令 ==========
        ax3.set_title('Control Commands (cmd_vel)')
        ax3.set_xlabel('Time Step')
        ax3.set_ylabel('Velocity')
        ax3.grid(True, alpha=0.3)

        # 记录控制指令历史
        if data['cmd_vel']:
            vx, vy, wz = data['cmd_vel']
            cmd_vel_history['vx'].append(vx)
            cmd_vel_history['vy'].append(vy)
            cmd_vel_history['wz'].append(wz)

            # 限制历史长度
            for key in cmd_vel_history:
                if len(cmd_vel_history[key]) > max_cmd_history:
                    cmd_vel_history[key].pop(0)

        # 绘制控制指令
        if cmd_vel_history['vx']:
            time_steps = range(len(cmd_vel_history['vx']))
            ax3.plot(time_steps, cmd_vel_history['vx'], 'r-', label='vx (m/s)', linewidth=2)
            ax3.plot(time_steps, cmd_vel_history['vy'], 'g-', label='vy (m/s)', linewidth=2)
            ax3.plot(time_steps, cmd_vel_history['wz'], 'b-', label='wz (rad/s)', linewidth=2)

            # 显示当前值
            if data['cmd_vel']:
                vx, vy, wz = data['cmd_vel']
                ax3.text(0.02, 0.98, f'vx: {vx:.3f} m/s\nvy: {vy:.3f} m/s\nwz: {wz:.3f} rad/s',
                        transform=ax3.transAxes, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        ax3.legend(loc='upper right', fontsize=8)

        # ========== 子图4: 位置误差 ==========
        ax4.set_title('Position Error (Current - Target)')
        ax4.set_xlabel('Time Step')
        ax4.set_ylabel('Error (m)')
        ax4.grid(True, alpha=0.3)

        # 计算误差
        left_error = None
        right_error = None

        if data['left_target'] and data['left_current']:
            left_error = np.linalg.norm(
                np.array(data['left_target']) - np.array(data['left_current'])
            )
            error_history['left'].append(left_error)

        if data['right_target'] and data['right_current']:
            right_error = np.linalg.norm(
                np.array(data['right_target']) - np.array(data['right_current'])
            )
            error_history['right'].append(right_error)

        # 限制历史长度
        for key in error_history:
            if len(error_history[key]) > max_cmd_history:
                error_history[key].pop(0)

        # 绘制误差曲线
        if error_history['left']:
            time_steps = range(len(error_history['left']))
            ax4.plot(time_steps, error_history['left'], 'r-', label='Left Arm Error', linewidth=2)

        if error_history['right']:
            time_steps = range(len(error_history['right']))
            ax4.plot(time_steps, error_history['right'], 'g-', label='Right Arm Error', linewidth=2)

        # 显示当前误差值
        if left_error is not None or right_error is not None:
            error_text = ''
            if left_error is not None:
                error_text += f'Left: {left_error:.4f} m\n'
            if right_error is not None:
                error_text += f'Right: {right_error:.4f} m'

            ax4.text(0.02, 0.98, error_text,
                    transform=ax4.transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.5))

        ax4.legend(loc='upper right', fontsize=8)

        plt.tight_layout()

    # 创建动画
    ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)  # 10 Hz更新

    try:
        # 显示图形
        plt.show()
    except KeyboardInterrupt:
        print("\n收到中断信号，正在关闭...")
    finally:
        # 停止ROS2 spin线程
        print("停止ROS2线程...")
        stop_event.set()
        ros_thread.join(timeout=2.0)

        # 清理ROS2资源
        print("清理ROS2资源...")
        try:
            visualizer.destroy_node()
        except:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

        print("可视化工具已关闭")


if __name__ == '__main__':
    main()
