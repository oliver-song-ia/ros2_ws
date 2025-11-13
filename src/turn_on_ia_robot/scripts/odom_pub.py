import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped
# import tf_transformations
import tf2_ros


# 定义机器人轮子的位置
wheel_positions = [
    (0.5, 0.25227),   # 前左
    (0.5, -0.25227),  # 前右
    (-0.5, 0.25227),   # 后左
    (-0.5, -0.25227)  # 后右
]
wheel_radius = 0.08  # 轮子半径，单位：米

# 初始化机器人位置和姿态
x = 0.0
y = 0.0
theta = 0.0

def forward_kinematics(thetas, wheel_speeds, wheel_positions, wheel_radius):
    """
    根据各轮的转向角和转速，计算机器人的整体运动参数（vx, vy, omega）
    
    :param thetas: 各轮转向角（弧度），列表形式 [theta1, theta2, theta3, theta4]
    :param wheel_speeds: 各轮驱动转速（rad/s），列表形式 [speed1, speed2, speed3, speed4]
    :param wheel_positions: 各轮位置（机器人坐标系下的坐标），列表形式 [(x1, y1), (x2, y2), ...]
    :param wheel_radius: 轮子半径（m）
    :return: 机器人整体线速度 vx, vy（m/s）和角速度 omega（rad/s）
    """
    # 将轮子转速转换为线速度（m/s）
    wheel_linear_speeds = [speed * wheel_radius for speed in wheel_speeds]
    
    # 构建线性方程组 A * [vx, vy, omega]^T = b
    A = []
    b = []
    
    for i in range(len(wheel_positions)):
        xi, yi = wheel_positions[i]
        si = wheel_linear_speeds[i]
        theta_i = thetas[i]
        
        # 轮子速度分解到机器人坐标系
        v_xi = si * math.cos(theta_i)
        v_yi = si * math.sin(theta_i)
        
        # 添加方程系数和常数项
        A.append([1, 0, -yi])  # vx方程：vx - omega*yi = v_xi
        b.append(v_xi)
        
        A.append([0, 1, xi])   # vy方程：vy + omega*xi = v_yi
        b.append(v_yi)
    
    # 转换为NumPy矩阵
    A = np.array(A)
    b = np.array(b)
    
    # 使用最小二乘法求解方程组：x = (A^T A)^(-1) A^T b
    x = np.linalg.lstsq(A, b, rcond=None)[0]
    
    vx, vy, omega = x[0], x[1], x[2]
    return vx, vy, omega

def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角转换为四元数
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        # 订阅 joint_states 主题
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        # 发布 odom 主题
        self.publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # self.tf_publisher = self.create_publisher(self)
        
        # 初始化上一次的时间戳
        self.last_time = self.get_clock().now().to_msg()

    def joint_states_callback(self, msg):
        global x, y, theta
        
        # 提取前左、前右、后左、后右的转向角和速度
        thetas = msg.position[:4]
        print("thetas: ", thetas)
        wheel_speeds = msg.velocity[-4:]
        print("wheel_speeds: ", wheel_speeds)
        
        # 获取当前时间戳
        current_time = msg.header.stamp
        
        # 计算时间间隔
        dt = (current_time.sec - self.last_time.sec) + (current_time.nanosec - self.last_time.nanosec) / 1e9
        
        # 计算机器人的整体运动参数
        vx, vy, omega = forward_kinematics(thetas, wheel_speeds, wheel_positions, wheel_radius)
        
        # 更新机器人的位置和姿态
        x += (vx * math.cos(theta) - vy * math.sin(theta)) * dt
        y += (vx * math.sin(theta) + vy * math.cos(theta)) * dt
        theta += omega * dt
        
        # 创建 Odometry 消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        
        # 设置位置
        x_sum =x*3.14
        y_sum =y*3.14
        odom_msg.pose.pose.position.x = x_sum
        odom_msg.pose.pose.position.y = y_sum
        odom_msg.pose.pose.position.z = 0.0
       
        # 设置姿态

        theta_sum = theta*3.14
        quaternion = euler_to_quaternion(0, 0, theta_sum)
        odom_msg.pose.pose.orientation = quaternion
        
        # 设置线速度和角速度
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = omega
        
        # 发布 Odometry 消息
        self.publisher.publish(odom_msg)
        print("x: ", x_sum, " y: ", y_sum, " theta: ", math.degrees(theta_sum))
        
        # 发布 TF 消息
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = x_sum
        t.transform.translation.y = y_sum
        t.transform.translation.z = 0.0
        # t.transform.rotation = euler_to_quaternion(0, 0, theta_sum)
        t.transform.rotation = quaternion
        self.tf_broadcaster.sendTransform(t)


        # 更新上一次的时间戳
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    
    odom_publisher = OdomPublisher()
    
    rclpy.spin(odom_publisher)
    
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
