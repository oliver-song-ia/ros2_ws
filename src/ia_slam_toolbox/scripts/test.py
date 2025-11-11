import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException
# 添加姿态计算所需的库
import tf_transformations as tf  # 新增导入

class TFGlobalPoseListener(Node):
    def __init__(self):
        super().__init__('tf_global_pose_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 定时查询变换（10Hz）
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        target_frame = 'map'
        source_frame = 'base_link'
        
        try:
            # 添加超时参数，解决时间同步问题
            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),  # 使用当前时间
                rclpy.duration.Duration(seconds=0.5)  # 超时时间设置为0.5秒
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            
            # 新增：提取旋转四元数并转换为欧拉角
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            roll, pitch, yaw = tf.euler_from_quaternion([qx, qy, qz, qw])
            
            # 更新日志输出，包含完整姿态信息
            self.get_logger().info(
                f"TF全局姿态: 位置(x: {x:.2f}, y: {y:.2f}, z: {z:.2f}), "
                f"姿态(roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f})"
            )
            
        except TransformException as ex:
            self.get_logger().warn(f"无法获取变换: {ex}")



def main(args=None):
    rclpy.init(args=args)
    node = TFGlobalPoseListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()