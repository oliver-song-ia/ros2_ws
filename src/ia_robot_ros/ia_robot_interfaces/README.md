# IA Robot Interfaces

ROS2 interface definitions (actions and services) for the IA Robot system.

## Interfaces

### Actions

#### MoveToStandby
Move robot arms to standby (lowered) position with progress feedback.

**Usage:**
```bash
ros2 action send_goal /move_to_standby ia_robot_interfaces/action/MoveToStandby "{}"
```

**Request:** Empty

**Result:**
- `success` (bool): Whether the movement completed successfully
- `message` (string): Status or error message

**Feedback:**
- `current_frame` (int32): Current interpolation frame
- `total_frames` (int32): Total number of frames
- `progress_percentage` (float32): Completion percentage (0-100)

**Notes:**
- Action will fail if joints are locked
- Movement is interpolated over 10 frames at 0.2s per frame (2 seconds total)
- Can be canceled during execution

**TODOs:**
- Dynamically adjust num_steps (currently 10) based on current pose and target pose

### Services

#### SetJointLock
Lock or unlock robot joints to prevent movement.

**Usage:**
```bash
# Lock joints
ros2 service call /set_joint_lock ia_robot_interfaces/srv/SetJointLock "{lock: true}"

# Unlock joints
ros2 service call /set_joint_lock ia_robot_interfaces/srv/SetJointLock "{lock: false}"
```

**Request:**
- `lock` (bool): true to lock joints, false to unlock

**Response:**
- `success` (bool): Whether the operation succeeded
- `message` (string): Confirmation or error message

**Notes:**
- When locked, the control loop skips command publishing
- MoveToStandby action will fail if joints are locked

## Building

```bash
colcon build --packages-select ia_robot_interfaces
source install/setup.bash
```

## Integration

See `mobile_ik_controller.py` for implementation examples of action server and service.
