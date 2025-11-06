#!/usr/bin/env python3
"""
Visualize robot odometry and velocity commands
Real-time plotting of robot trajectory comparison
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
import threading

# Global variable to keep animation reference
_animation = None

class OdomVisualizer(Node):
    def __init__(self):
        super().__init__('odom_visualizer')

        # Subscribe to topics
        self.odom_sub = self.create_subscription(
            Odometry,
            '/swerve_drive_controller/odom',
            self.odom_callback,
            10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Data storage - no maxlen to keep all history
        # Odom trajectory
        self.odom_x = deque()
        self.odom_y = deque()
        self.current_theta = 0.0

        # Record first position as origin (relative coordinates)
        self.odom_origin_x = None
        self.odom_origin_y = None

        # Cmd_vel integrated trajectory
        self.cmd_integrated_x = 0.0
        self.cmd_integrated_y = 0.0
        self.cmd_integrated_theta = 0.0
        self.cmd_integrated_x_history = deque()
        self.cmd_integrated_y_history = deque()
        self.last_cmd_time = None

        # Store current cmd_vel for continuous integration
        self.current_cmd_vx = 0.0
        self.current_cmd_vy = 0.0
        self.current_cmd_wz = 0.0
        self.last_cmd_vel_time = None  # Track last cmd_vel message time

        # Create timer for continuous integration (50Hz)
        self.integration_timer = self.create_timer(0.02, self.integration_callback)

        self.get_logger().info('Odom Visualizer started!')
        self.get_logger().info('Subscribed to: /swerve_drive_controller/odom, /cmd_vel')

    def odom_callback(self, msg):
        """Handle odometry data"""
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Set origin on first message
        if self.odom_origin_x is None:
            self.odom_origin_x = x
            self.odom_origin_y = y
            self.get_logger().info(f'Odom origin set to: ({x:.3f}, {y:.3f})')

        # Extract orientation
        orientation = msg.pose.pose.orientation
        self.current_theta = self.quaternion_to_yaw(orientation)

        # Extract linear and angular velocities
        linear_vel = msg.twist.twist.linear
        angular_vel = msg.twist.twist.angular

        # Print odom data
        rel_x = x - self.odom_origin_x
        rel_y = y - self.odom_origin_y
        self.get_logger().info(
            f'[ODOM] Pos: ({rel_x:.3f}, {rel_y:.3f}) m, '
            f'Theta: {np.degrees(self.current_theta):.1f}°, '
            f'Vel: vx={linear_vel.x:.3f}, vy={linear_vel.y:.3f}, '
            f'wz={angular_vel.z:.3f} rad/s'
        )

        # Store data relative to origin
        self.odom_x.append(rel_x)
        self.odom_y.append(rel_y)

    def cmd_vel_callback(self, msg):
        """Handle velocity command data - just store the current velocity"""
        # Store current velocities for continuous integration
        self.current_cmd_vx = msg.linear.x / np.pi
        self.current_cmd_vy = msg.linear.y / np.pi
        self.current_cmd_wz = msg.angular.z / np.pi
        # Update timestamp when receiving new cmd_vel
        self.last_cmd_vel_time = self.get_clock().now()

        # Print cmd_vel data
        self.get_logger().info(
            f'[CMD_VEL] Linear: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, vz={msg.linear.z:.3f} m/s, '
            f'Angular: wx={msg.angular.x:.3f}, wy={msg.angular.y:.3f}, wz={msg.angular.z:.3f} rad/s'
        )

    def integration_callback(self):
        """Continuously integrate cmd_vel at fixed rate (50Hz)"""
        current_time = self.get_clock().now()

        # Check cmd_vel timeout (0.1s = 1/10Hz)
        if self.last_cmd_vel_time is not None:
            cmd_age = (current_time - self.last_cmd_vel_time).nanoseconds / 1e9
            if cmd_age > 0.1:  # Same as controller timeout
                # Timeout! Set velocity to 0
                self.current_cmd_vx = 0.0
                self.current_cmd_vy = 0.0
                self.current_cmd_wz = 0.0

        # Integrate cmd_vel to get expected position
        if self.last_cmd_time is not None:
            dt = (current_time - self.last_cmd_time).nanoseconds / 1e9

            # Integrate in world frame
            cos_theta = np.cos(self.cmd_integrated_theta)
            sin_theta = np.sin(self.cmd_integrated_theta)

            vx_global = self.current_cmd_vx * cos_theta - self.current_cmd_vy * sin_theta
            vy_global = self.current_cmd_vx * sin_theta + self.current_cmd_vy * cos_theta

            self.cmd_integrated_x += vx_global * dt
            self.cmd_integrated_y += vy_global * dt
            self.cmd_integrated_theta += self.current_cmd_wz * dt

            # Normalize angle
            self.cmd_integrated_theta = np.arctan2(
                np.sin(self.cmd_integrated_theta),
                np.cos(self.cmd_integrated_theta)
            )

            # Store integrated position
            self.cmd_integrated_x_history.append(self.cmd_integrated_x)
            self.cmd_integrated_y_history.append(self.cmd_integrated_y)

        self.last_cmd_time = current_time

    @staticmethod
    def quaternion_to_yaw(q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)


def animate_plots(visualizer):
    """Create animation visualization with two trajectory plots"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    # Initialize plot elements once
    ax1.set_xlabel('X (m)', fontsize=14)
    ax1.set_ylabel('Y (m)', fontsize=14)
    ax1.set_title('Odometry Trajectory (Actual)', fontsize=16, fontweight='bold', color='blue')
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')

    ax2.set_xlabel('X (m)', fontsize=14)
    ax2.set_ylabel('Y (m)', fontsize=14)
    ax2.set_title('Cmd_vel Integrated Trajectory (Expected)', fontsize=16, fontweight='bold', color='red')
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')

    # Store line objects for updating
    odom_line, = ax1.plot([], [], 'b-', linewidth=2, alpha=0.7, label='Trajectory')
    odom_start, = ax1.plot([], [], 'go', markersize=12, label='Start', zorder=5)
    odom_current, = ax1.plot([], [], 'ro', markersize=12, label='Current', zorder=5)
    odom_arrow = None

    cmd_line, = ax2.plot([], [], 'r-', linewidth=2, alpha=0.7, label='Trajectory')
    cmd_start, = ax2.plot([], [], 'go', markersize=12, label='Start', zorder=5)
    cmd_current, = ax2.plot([], [], 'ro', markersize=12, label='Current', zorder=5)
    cmd_arrow = None

    ax1.legend(loc='upper right', fontsize=12)
    ax2.legend(loc='upper right', fontsize=12)

    def init():
        """Initialize plots"""
        return []

    def update(frame):
        """Update plots without clearing"""
        nonlocal odom_arrow, cmd_arrow

        # Update Plot 1: Odom trajectory
        if len(visualizer.odom_x) > 0:
            odom_x_list = list(visualizer.odom_x)
            odom_y_list = list(visualizer.odom_y)

            # Update trajectory line
            odom_line.set_data(odom_x_list, odom_y_list)

            # Update start and current position markers
            odom_start.set_data([0], [0])
            odom_current.set_data([odom_x_list[-1]], [odom_y_list[-1]])

            # Remove old arrow if exists
            if odom_arrow is not None:
                odom_arrow.remove()

            # Draw current heading (adaptive arrow size)
            x_range = max(odom_x_list) - min(odom_x_list) if len(odom_x_list) > 1 else 1.0
            y_range = max(odom_y_list) - min(odom_y_list) if len(odom_y_list) > 1 else 1.0
            plot_range = max(x_range, y_range, 0.1)  # Minimum 0.1m
            arrow_len = plot_range * 0.1  # Arrow is 10% of plot range

            dx = arrow_len * np.cos(visualizer.current_theta)
            dy = arrow_len * np.sin(visualizer.current_theta)
            odom_arrow = ax1.arrow(odom_x_list[-1], odom_y_list[-1], dx, dy,
                     head_width=arrow_len*0.4, head_length=arrow_len*0.3,
                     fc='red', ec='darkred', linewidth=2, zorder=6)

            # Auto-scale the axes
            ax1.relim()
            ax1.autoscale_view()

        # Update Plot 2: Cmd_vel integrated trajectory
        if len(visualizer.cmd_integrated_x_history) > 0:
            cmd_x_list = list(visualizer.cmd_integrated_x_history)
            cmd_y_list = list(visualizer.cmd_integrated_y_history)

            # Update trajectory line
            cmd_line.set_data(cmd_x_list, cmd_y_list)

            # Update start and current position markers
            cmd_start.set_data([0], [0])
            cmd_current.set_data([cmd_x_list[-1]], [cmd_y_list[-1]])

            # Remove old arrow if exists
            if cmd_arrow is not None:
                cmd_arrow.remove()

            # Draw current heading (adaptive arrow size)
            x_range = max(cmd_x_list) - min(cmd_x_list) if len(cmd_x_list) > 1 else 1.0
            y_range = max(cmd_y_list) - min(cmd_y_list) if len(cmd_y_list) > 1 else 1.0
            plot_range = max(x_range, y_range, 0.1)  # Minimum 0.1m
            arrow_len = plot_range * 0.1  # Arrow is 10% of plot range

            dx = arrow_len * np.cos(visualizer.cmd_integrated_theta)
            dy = arrow_len * np.sin(visualizer.cmd_integrated_theta)
            cmd_arrow = ax2.arrow(cmd_x_list[-1], cmd_y_list[-1], dx, dy,
                     head_width=arrow_len*0.4, head_length=arrow_len*0.3,
                     fc='red', ec='darkred', linewidth=2, zorder=6)

            # Auto-scale the axes
            ax2.relim()
            ax2.autoscale_view()

        return []

    # Create animation
    global _animation
    _animation = animation.FuncAnimation(
        fig, update, init_func=init,
        interval=50, blit=False, cache_frame_data=False, save_count=0
    )

    plt.tight_layout()
    plt.show()


def main(args=None):
    rclpy.init(args=args)
    visualizer = OdomVisualizer()

    # Run ROS2 spin in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(visualizer,), daemon=True)
    ros_thread.start()

    # Run matplotlib animation in main thread
    try:
        animate_plots(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
