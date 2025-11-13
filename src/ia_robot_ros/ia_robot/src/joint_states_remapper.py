#!/usr/bin/env python3
"""
Joint States Remapper Node

This node subscribes to /joint_states and republishes the data to /robot/joint_states
with the joint names remapped for the swerve drive controller.

Mapping:
- Steering joints (position control):
  Leg_front_left_2  -> lf_steer_joint
  Leg_front_right_2 -> rf_steer_joint
  Leg_back_left_2   -> lb_steer_joint
  Leg_back_right_2  -> rb_steer_joint

- Wheel joints (velocity control):
  Leg_front_left_1  -> lf_wheel_joint
  Leg_front_right_1 -> rf_wheel_joint
  Leg_back_left_1   -> lb_wheel_joint
  Leg_back_right_1  -> rb_wheel_joint
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

K = 0.6634


class JointStatesRemapper(Node):
    def __init__(self):
        super().__init__('joint_states_remapper')

        # Joint name mapping: original_name -> new_name
        self.joint_mapping = {
            # Steering joints (from original to expected)
            'Leg_front_left_2':  'lf_steer_joint',
            'Leg_front_right_2': 'rf_steer_joint',
            'Leg_back_left_2':   'lb_steer_joint',
            'Leg_back_right_2':  'rb_steer_joint',

            # Wheel joints (from original to expected)
            'Leg_front_left_1':  'lf_wheel_joint',
            'Leg_front_right_1': 'rf_wheel_joint',
            'Leg_back_left_1':   'lb_wheel_joint',
            'Leg_back_right_1':  'rb_wheel_joint',
        }

        # Expected output order for consistency
        self.output_order = [
            'lf_steer_joint',
            'rf_steer_joint',
            'lb_steer_joint',
            'rb_steer_joint',
            'lf_wheel_joint',
            'rf_wheel_joint',
            'lb_wheel_joint',
            'rb_wheel_joint',
        ]

        # Subscribe to original joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publish to remapped joint_states
        self.publisher = self.create_publisher(
            JointState,
            '/robot/joint_states',
            10
        )

        self.get_logger().info('Joint States Remapper started')
        self.get_logger().info('Subscribing to: /joint_states')
        self.get_logger().info('Publishing to: /robot/joint_states')

    def joint_state_callback(self, msg):
        """
        Callback function that remaps joint names and republishes

        Steering joints (Leg_*_2) use position field
        Wheel joints (Leg_*_1) use velocity field
        """
        # Create output message
        output_msg = JointState()
        output_msg.header = msg.header

        # Create dictionaries to store data by original joint name
        position_dict = {}
        velocity_dict = {}
        effort_dict = {}

        # Extract data from input message
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_mapping:
                # Store position
                if i < len(msg.position):
                    position_dict[joint_name] = msg.position[i]

                # Store velocity
                if i < len(msg.velocity):
                    velocity_dict[joint_name] = msg.velocity[i]

                # Store effort (if available)
                if i < len(msg.effort):
                    effort_dict[joint_name] = msg.effort[i]

        # Build output arrays in the desired order
        output_msg.name = []
        output_msg.position = []
        output_msg.velocity = []
        output_msg.effort = []

        # Steering joint names (use position field)
        steering_joints = ['Leg_front_left_2', 'Leg_front_right_2',
                          'Leg_back_left_2', 'Leg_back_right_2']

        # Wheel joint names (use velocity field)
        wheel_joints = ['Leg_front_left_1', 'Leg_front_right_1',
                       'Leg_back_left_1', 'Leg_back_right_1']

        # Front wheels need velocity reversed
        front_wheel_joints = ['Leg_front_left_1', 'Leg_front_right_1']

        for original_name in self.output_order:
            # Find the original name that maps to this output name
            original_name_key = None
            for orig, new in self.joint_mapping.items():
                if new == original_name:
                    original_name_key = orig
                    break

            # Only add joints that were found in the input message
            if original_name_key and original_name_key in position_dict:
                output_msg.name.append(original_name)

                # For steering joints: use position for both position and velocity
                if original_name_key in steering_joints:
                    output_msg.position.append(-position_dict.get(original_name_key, 0.0))
                    output_msg.velocity.append(0.0)  # Steering velocity not used

                # For wheel joints: position=0, velocity from velocity field
                elif original_name_key in wheel_joints:
                    output_msg.position.append(0.0)  # Wheel position not used

                    # Front wheels need velocity reversed
                    vel = velocity_dict.get(original_name_key, 0.0) * K
                    if original_name_key in front_wheel_joints:
                        vel = -vel  # Reverse front wheel velocity

                    output_msg.velocity.append(vel)

                else:
                    # Fallback
                    output_msg.position.append(position_dict.get(original_name_key, 0.0))
                    output_msg.velocity.append(velocity_dict.get(original_name_key, 0.0))

                # Effort is optional
                if original_name_key in effort_dict:
                    output_msg.effort.append(effort_dict[original_name_key])

        # Publish the remapped message
        self.publisher.publish(output_msg)

        # Log periodically (every 100 messages)
        if not hasattr(self, 'msg_count'):
            self.msg_count = 0
        self.msg_count += 1

        # if self.msg_count % 100 == 0:
        #     self.get_logger().info(
        #         f'Remapped {len(output_msg.name)} joints: {output_msg.name}'
        #     )


def main(args=None):
    rclpy.init(args=args)

    node = JointStatesRemapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()