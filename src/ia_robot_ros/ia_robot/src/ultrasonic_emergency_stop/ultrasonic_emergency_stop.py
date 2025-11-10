#!/usr/bin/env python3
"""
Ultrasonic Emergency Stop Node
Acts as the last line of defense, immediately stopping the robot when ultrasonic sensor detects obstacles too close
Similar to collision sensor functionality in robot vacuums
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class UltrasonicEmergencyStop(Node):
    """
    Ultrasonic Emergency Stop Node
    
    Features:
    1. Subscribe to ultrasonic sensor data (sensor_msgs/Range)
    2. Immediately publish zero velocity command when detected distance is less than safety threshold
    3. Publish emergency stop status for other nodes to reference
    """
    
    def __init__(self):
        super().__init__('ultrasonic_emergency_stop')
        
        # Declare parameters
        self.declare_parameter('safety_threshold', 0.15)  # Safety distance threshold (m)
        self.declare_parameter('hysteresis_threshold', 0.20)  # Hysteresis threshold to prevent oscillation (m)
        
        # Eight corner sensors - 2 per corner (forward-facing and side-facing)
        self.declare_parameter('ultrasonic_front_left_forward_topic', '/ultrasonic/front_left_forward')
        self.declare_parameter('ultrasonic_front_left_side_topic', '/ultrasonic/front_left_side')
        self.declare_parameter('ultrasonic_front_right_forward_topic', '/ultrasonic/front_right_forward')
        self.declare_parameter('ultrasonic_front_right_side_topic', '/ultrasonic/front_right_side')
        self.declare_parameter('ultrasonic_rear_left_forward_topic', '/ultrasonic/rear_left_forward')
        self.declare_parameter('ultrasonic_rear_left_side_topic', '/ultrasonic/rear_left_side')
        self.declare_parameter('ultrasonic_rear_right_forward_topic', '/ultrasonic/rear_right_forward')
        self.declare_parameter('ultrasonic_rear_right_side_topic', '/ultrasonic/rear_right_side')
        
        self.declare_parameter('emergency_cmd_vel_topic', '/cmd_vel_emergency')
        self.declare_parameter('emergency_status_topic', '/emergency_stop_status')
        self.declare_parameter('publish_rate', 20.0)  # Publish rate (Hz)
        self.declare_parameter('sensor_timeout', 1.0)  # Sensor timeout (s)
        self.declare_parameter('enable_emergency_stop', True)  # Whether to enable emergency stop functionality
        
        # Get parameters
        self.safety_threshold = self.get_parameter('safety_threshold').value
        self.hysteresis_threshold = self.get_parameter('hysteresis_threshold').value
        
        # Get all eight sensor topics
        self.sensor_topics = {
            'front_left_forward': self.get_parameter('ultrasonic_front_left_forward_topic').value,
            'front_left_side': self.get_parameter('ultrasonic_front_left_side_topic').value,
            'front_right_forward': self.get_parameter('ultrasonic_front_right_forward_topic').value,
            'front_right_side': self.get_parameter('ultrasonic_front_right_side_topic').value,
            'rear_left_forward': self.get_parameter('ultrasonic_rear_left_forward_topic').value,
            'rear_left_side': self.get_parameter('ultrasonic_rear_left_side_topic').value,
            'rear_right_forward': self.get_parameter('ultrasonic_rear_right_forward_topic').value,
            'rear_right_side': self.get_parameter('ultrasonic_rear_right_side_topic').value,
        }
        
        self.emergency_cmd_vel_topic = self.get_parameter('emergency_cmd_vel_topic').value
        self.emergency_status_topic = self.get_parameter('emergency_status_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.sensor_timeout = self.get_parameter('sensor_timeout').value
        self.enable_emergency_stop = self.get_parameter('enable_emergency_stop').value
        
        # Parameter validation
        if self.safety_threshold >= self.hysteresis_threshold:
            self.get_logger().warn(
                f'Hysteresis threshold ({self.hysteresis_threshold}m) should be greater than '
                f'safety threshold ({self.safety_threshold}m). Adjusting...'
            )
            self.hysteresis_threshold = self.safety_threshold + 0.05
        
        # State variables - Store range and timestamp for each sensor
        self.sensor_ranges = {key: float('inf') for key in self.sensor_topics.keys()}
        self.sensor_last_times = {key: None for key in self.sensor_topics.keys()}
        self.emergency_stop_active = False
        
        # QoS configuration - Use BEST_EFFORT to ensure low latency
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers - Create one for each sensor
        self.subscribers = {}
        for sensor_name, topic in self.sensor_topics.items():
            self.subscribers[sensor_name] = self.create_subscription(
                Range,
                topic,
                lambda msg, name=sensor_name: self.sensor_callback(msg, name),
                qos_profile
            )
        
        # Publishers
        # Emergency stop velocity command - High priority, used for twist_mux
        self.emergency_cmd_vel_pub = self.create_publisher(
            Twist,
            self.emergency_cmd_vel_topic,
            10
        )
        
        # Emergency stop status - For other nodes to monitor
        self.emergency_status_pub = self.create_publisher(
            Bool,
            self.emergency_status_topic,
            10
        )
        
        # Timer - Periodically check and publish
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        sensor_list = '\n'.join([f'  {name}: {topic}' for name, topic in self.sensor_topics.items()])
        self.get_logger().info(
            f'Ultrasonic emergency stop node started\n'
            f'  Safety threshold: {self.safety_threshold}m\n'
            f'  Hysteresis threshold: {self.hysteresis_threshold}m\n'
            f'  Number of sensors: {len(self.sensor_topics)}\n'
            f'{sensor_list}\n'
            f'  Emergency stop function: {"Enabled" if self.enable_emergency_stop else "Disabled"}'
        )
    
    def sensor_callback(self, msg: Range, sensor_name: str):
        """Generic sensor callback for all eight sensors"""
        self.sensor_ranges[sensor_name] = msg.range
        self.sensor_last_times[sensor_name] = self.get_clock().now()
        
        # Check validity
        if msg.range < msg.min_range or msg.range > msg.max_range:
            # Invalid reading, use inf to indicate no obstacle
            self.sensor_ranges[sensor_name] = float('inf')
    
    def is_sensor_data_valid(self):
        """Check if sensor data is valid (not timed out)"""
        current_time = self.get_clock().now()
        timeout_duration = rclpy.duration.Duration(seconds=self.sensor_timeout)
        
        # Check all sensors - at least one must have valid data
        valid_sensors = []
        timed_out_sensors = []
        
        for sensor_name, last_time in self.sensor_last_times.items():
            if last_time is None:
                continue  # Sensor hasn't received data yet
            
            if (current_time - last_time) > timeout_duration:
                timed_out_sensors.append(sensor_name)
                current_sec = current_time.nanoseconds / 1e9
                last_sec = last_time.nanoseconds / 1e9
                self.get_logger().info(
                    f'Sensor {sensor_name} data timed out - current: {current_sec:.3f}s, last: {last_sec:.3f}s'
                )
            else:
                valid_sensors.append(sensor_name)
        
        # Log timeouts
        if timed_out_sensors:
            self.get_logger().warning(
                f'Ultrasonic sensor(s) timed out: {", ".join(timed_out_sensors)}',
                throttle_duration_sec=1.0
            )
        
        # Return True if at least one sensor has valid data
        return len(valid_sensors) > 0
    
    def check_emergency_condition(self):
        """
        Check if emergency stop is needed
        Uses hysteresis logic to prevent oscillation:
        - When distance < safety_threshold, trigger emergency stop
        - When distance > hysteresis_threshold, release emergency stop
        - Between the two, maintain current state
        """
        if not self.is_sensor_data_valid():
            # When sensor data is invalid, to be conservative, don't publish emergency stop command
            # Let nav2's other safety mechanisms handle it
            return False
        
        # Find minimum range across all sensors
        min_range = min(self.sensor_ranges.values())
        
        # Find which sensor(s) detected the closest obstacle
        closest_sensors = [name for name, range_val in self.sensor_ranges.items() 
                          if range_val == min_range and range_val != float('inf')]
        
        if min_range < self.safety_threshold:
            # Trigger emergency stop
            if not self.emergency_stop_active:
                sensor_info = f" (sensor: {', '.join(closest_sensors)})" if closest_sensors else ""
                self.get_logger().warn(
                    f'🚨 Emergency stop triggered! Obstacle detected at distance: {min_range:.3f}m '
                    f'(threshold: {self.safety_threshold}m){sensor_info}'
                )
            self.emergency_stop_active = True
            return True
        elif min_range > self.hysteresis_threshold:
            # Release emergency stop
            if self.emergency_stop_active:
                self.get_logger().info(
                    f'✓ Emergency stop released, resuming normal operation (distance: {min_range:.3f}m)'
                )
            self.emergency_stop_active = False
            return False
        else:
            # Within hysteresis range, maintain current state
            return self.emergency_stop_active
    
    def timer_callback(self):
        """Timer callback - Periodically check and publish commands"""
        if not self.enable_emergency_stop:
            return
        
        should_stop = self.check_emergency_condition()
        
        # Publish emergency stop status
        status_msg = Bool()
        status_msg.data = should_stop
        self.emergency_status_pub.publish(status_msg)
        
        # If emergency stop is needed, publish zero velocity command
        if should_stop:
            stop_cmd = Twist()
            # Set all velocity components to 0
            stop_cmd.linear.x = 0.0
            stop_cmd.linear.y = 0.0
            stop_cmd.linear.z = 0.0
            stop_cmd.angular.x = 0.0
            stop_cmd.angular.y = 0.0
            stop_cmd.angular.z = 0.0
            
            self.emergency_cmd_vel_pub.publish(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = UltrasonicEmergencyStop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Exception: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
