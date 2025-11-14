#!/usr/bin/env python3
"""
发布8条射线区域的LaserScan消息
射线区域（在激光雷达坐标系中）：
- +35°~+65° (1条射线区域)
- -35°~-65° (1条射线区域)
- +152.5°~+167.5° (1条射线区域)
- -152.5°~-167.5° (1条射线区域)
共4个区域，每个区域包含多个射线点
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class EightRaysPublisher(Node):
    def __init__(self):
        super().__init__('eight_rays_publisher')

        # 创建发布者
        self.publisher_ = self.create_publisher(LaserScan, 'eight_rays_scan', 10)

        # 发布频率 (Hz)
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_scan)

        # 定义8条射线区域的角度范围（度）
        # 每个区域用 (start, end) 表示
        self.ray_ranges_deg = [
            (15.0, 35.0),      # 右前方区域
            (-35.0, -15.0),    # 左前方区域
            (145.0, 170.0),    # 右后方区域
            (-170.0, -145.0),  # 左后方区域
        ]

        # 激光雷达参数
        self.angle_min = math.radians(-180.0)  # -π
        self.angle_max = math.radians(180.0)   # +π
        self.angle_increment = math.radians(0.25)  # 0.25度角度分辨率
        self.range_min = 0.1
        self.range_max = 10.0
        self.range_value = 2.0  # 射线的距离值

        # 计算需要的射线总数
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1

        self.get_logger().info(f'Eight Rays Publisher started')
        self.get_logger().info(f'Ray ranges (degrees): {self.ray_ranges_deg}')
        self.get_logger().info(f'Publishing on topic: eight_rays_scan')
        self.get_logger().info(f'Total scan points: {self.num_ranges}')

    def publish_scan(self):
        """发布LaserScan消息"""
        scan = LaserScan()

        # 设置header
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_footprint'  # 激光雷达坐标系

        # 设置扫描参数
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.timer_period
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # 初始化所有射线为无穷大（无障碍物）
        scan.ranges = [float('inf')] * self.num_ranges

        # 在指定的角度区域内设置有效距离
        for i in range(self.num_ranges):
            # 计算当前索引对应的角度（弧度）
            current_angle_rad = self.angle_min + i * self.angle_increment
            # 转换为度
            current_angle_deg = math.degrees(current_angle_rad)

            # 检查当前角度是否在任何一个射线区域内
            for start_deg, end_deg in self.ray_ranges_deg:
                if start_deg <= end_deg:
                    # 正常区间
                    if start_deg <= current_angle_deg <= end_deg:
                        scan.ranges[i] = self.range_value
                        break
                else:
                    # 跨越-180/+180边界的区间（如-167.5到-152.5）
                    if current_angle_deg >= start_deg or current_angle_deg <= end_deg:
                        scan.ranges[i] = self.range_value
                        break

        # 发布消息
        self.publisher_.publish(scan)


def main(args=None):
    rclpy.init(args=args)

    node = EightRaysPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
