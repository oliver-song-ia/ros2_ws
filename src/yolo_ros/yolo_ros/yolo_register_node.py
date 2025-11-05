import cv2
import numpy as np
from typing import List, Tuple

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState
import json
import message_filters
from cv_bridge import CvBridge
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
import csv
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped
from yolo_msgs.msg import Detection
from yolo_msgs.msg import DetectionArray
from yolo_msgs.msg import KeyPoint3D
from yolo_msgs.msg import KeyPoint3DArray
from yolo_msgs.msg import BoundingBox3D
from rclpy.qos import qos_profile_sensor_data

class YoloSlamNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("objects_register_node")


        # aux
        self.tf_buffer = Buffer()
        self.cv_bridge = CvBridge()
        self.global_objects=[]

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Configuring...")


        

        super().on_configure(state)
        self.get_logger().info(f"[{self.get_name()}] Configured")

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Activating...")
        self.get_logger().info(f"[{self.get_name()}] subscribing to detections_3d")

        # Initialize subscriptions and synchronizers
        # self.detections_sub = message_filters.Subscriber(
        #     self, DetectionArray, "detections_3d"
        # )
        self.create_subscription( DetectionArray,
            "detections_3d",
            self.on_detections,
            qos_profile_sensor_data)
        self.get_logger().info(f"[{self.get_name()}] subscribed to detections_3d")
        

        # self._synchronizer = message_filters.ApproximateTimeSynchronizer(
        #     (self.detections_sub), 10, 0.5
        # )
      

        # Call the base class method and check for success
        base_return = super().on_activate(state)
        if base_return != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error(f"[{self.get_name()}] Failed to activate.")
            return base_return  # Return the failure status if base transition fails

        self.get_logger().info(f"[{self.get_name()}] Activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Deactivating...")


        self.destroy_subscription(self.detections_sub.sub)

        del self._synchronizer

        super().on_deactivate(state)
        self.get_logger().info(f"[{self.get_name()}] Deactivated")

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Cleaning up...")

        del self.tf_listener

        self.destroy_publisher(self._pub)

        super().on_cleanup(state)
        self.get_logger().info(f"[{self.get_name()}] Cleaned up")

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.save_objects_to_csv()  # Uncomment for CSV file
        self.get_logger().info(f"[{self.get_name()}] Shutting down...")
        super().on_cleanup(state)
        self.get_logger().info(f"[{self.get_name()}] Shutted down")
        # self.save_objects_to_json()  # For JSON file
        
        self.logger().info(f"Objects saved to CSV")
        return TransitionCallbackReturn.SUCCESS

    def on_detections(
        self,
        detections_msg: DetectionArray,
    ) -> None:

        self.process_detections(detections_msg)

    def process_detections(self, dection_msg):
        for det in dection_msg.detections:
            center = det.bbox3d.center.position
            size = det.bbox3d.size
            class_name = det.class_name  # Using 'class_name' for the label
            center_xyz = (center.x, center.y, center.z)
            size_xyz = (size.x, size.y, size.z)

            if not self.is_duplicate(center_xyz, size_xyz):
                self.global_objects.append({
                    'label': class_name,  # Use 'class_name' here
                    'center': center_xyz,
                    'size': size_xyz
                })
                self.get_logger().info(
                    f"New object {class_name} added at center: ({center.x:.2f}, {center.y:.2f}, {center.z:.2f})"
                )

    def is_duplicate(self, new_center, new_size):
        for obj in self.global_objects:
            if self.boxes_overlap(obj['center'], obj['size'], new_center, new_size):
                return True
        # Check for distance threshold
        for obj in self.global_objects:
            if self.check_for_collision(obj['center'], new_center, dist_threshold=0.5):
                return True
        return False

    def boxes_overlap(self, center1, size1, center2, size2):
        """Returns True if two AABBs overlap"""
        for i in range(3):  # x, y, z
            min1 = center1[i] - size1[i] / 2.0
            max1 = center1[i] + size1[i] / 2.0
            min2 = center2[i] - size2[i] / 2.0
            max2 = center2[i] + size2[i] / 2.0
            if max1 < min2 or max2 < min1:
                return False  # no overlap along this axis
        return True
    
    def check_for_collision(self, center1,center2, dist_threshold=0.25):
        """Check if two centers are within a certain distance threshold"""
        dist = np.linalg.norm(np.array(center1) - np.array(center2))
        return dist < dist_threshold

    def save_objects_to_json(self):
        """Save the global objects to a JSON file"""
        with open('./objects.json', 'w') as json_file:
            json.dump(self.global_objects, json_file, indent=4)
        self.get_logger().info("Objects saved to JSON")

    def save_objects_to_csv(self):
        """Save the global objects to a CSV file"""
        with open('./objects.csv', mode='w', newline='') as csv_file:
            fieldnames = ['label','center_x', 'center_y', 'center_z', 'size_x', 'size_y', 'size_z']
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            self.get_logger().info(f"Writing {len(self.global_objects)} objects to CSV")
            for obj in self.global_objects:
                print(obj)
                writer.writerow({
                    'label': obj['label'],  # Write the label (class_name)
                    'center_x': obj['center'][0],
                    'center_y': obj['center'][1],
                    'center_z': obj['center'][2],
                    'size_x': obj['size'][0],
                    'size_y': obj['size'][1],
                    'size_z': obj['size'][2]
                })
        self.get_logger().info("Objects saved to CSV")

def main():
    import json
    import os

    rclpy.init()
    node = YoloSlamNode()

    try:
        node.trigger_configure()
        node.trigger_activate()
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, saving global objects...")

    finally:
        # Save global objects here
        if hasattr(node, "global_objects"):

            try:
                node.save_objects_to_csv()
            except Exception as e:
                node.get_logger().error(f"Failed to save global objects: {e}")
        else:
            node.get_logger().warn("No global_objects attribute found on the node")

        node.destroy_node()
        rclpy.shutdown()