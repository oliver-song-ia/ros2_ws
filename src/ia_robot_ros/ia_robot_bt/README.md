# IA Robot Behavior Tree - Simplified

Simplified behavior tree implementation for mission coordination in the IA robot system.

## Overview

This package implements the mission coordinator using **behavior trees** with a **simplified workflow**. The tree passively monitors Nav2's navigation and only manages the manipulation phase actively.

### Simplified Workflow

```
1. Listen to /goal_pose
   ↓
2. Goal received → Nav2 handles navigation automatically
   ↓  
3. Monitor Nav2 status topic
   ↓
4. Navigation succeeds → Start manipulation
   ↓
5. Wait for manipulation complete
   ↓
6. Stop manipulation
   ↓
7. Loop back to step 1
```

## Behavior Tree Structure

```
Root (Sequence - loops forever)
├── Wait for Goal Pose (/goal_pose topic)
├── Wait for Navigation Complete (monitor Nav2 status)
├── Start Manipulation (IK + mocap)
├── Wait for Manipulation Complete (timeout or process exit)
└── Stop Manipulation (cleanup)
```

## Planner Behavior Tree Structure

```
Planner Mode Selector (Selector with memory)
├── Rest Mode Sequence
│   ├── Is Rest Requested? (ModeSelector condition)
│   └── Execute Rest Mode (RestModeAction)
├── Approach Mode Sequence
│   ├── Is Approach Requested? (ModeSelector condition)
│   └── Execute Approach Mode (ApproachModeAction)
├── Assist Mode Sequence
│   ├── Is Assist Requested? (ModeSelector condition)
│   └── Execute Assist Mode (AssistModeAction)
└── Retreat Mode Sequence
    ├── Is Retreat Requested? (ModeSelector condition)
    └── Execute Retreat Mode (RetreatModeAction)
```

### How It Works

1. **Selector with Memory**: The root selector remembers which mode is currently active
2. **Mode Sequences**: Each mode is a sequence that:
   - First checks if that mode is requested (via `ModeSelector`)
   - If yes, executes the mode action
3. **Mode Actions**: Return `RUNNING` continuously while active
4. **Mode Transitions**: When mode changes via `/set_mode` topic:
   - Current mode action detects the change and returns `SUCCESS`
   - Selector re-evaluates and switches to new mode sequence

## Integration with Mission BT

The planner subtree is integrated into the manipulation phase using a `Parallel` composite:

```
Manipulation Phase (Parallel - SuccessOnOne)
├── Planner Subtree (RUNNING continuously, managing modes)
└── Wait Manipulation Complete (SUCCESS on timeout or process end)
```

This allows the planner to run continuously while waiting for manipulation to complete.

### ROS Interface

#### Subscribed Topics
- `/pose_detection` (MarkerArray) - Human skeleton joints
- `/current_poses` (PoseArray) - Current robot EEF poses
- `/set_mode` (String) - Mode change commands

#### Published Topics
- `/target_eef_poses` (PoseArray) - Target EEF positions
- `/mode` (String) - Current planner mode

#### Mode Commands
Send mode change commands:
```bash
ros2 topic pub /set_mode std_msgs/msg/String "data: 'rest'" --once
ros2 topic pub /set_mode std_msgs/msg/String "data: 'approach'" --once
ros2 topic pub /set_mode std_msgs/msg/String "data: 'assist'" --once
ros2 topic pub /set_mode std_msgs/msg/String "data: 'retreat'" --once
```

### Blackboard Keys

The planner uses the following blackboard keys:

| Key | Type | Description |
|-----|------|-------------|
| `planner_mode` | str | Current mode (rest/approach/assist/retreat) |
| `planner_mode_requested` | str | Requested mode via /set_mode |
| `planner_joints` | np.array | Latest human skeleton joints |
| `planner_human_pos` | np.array | Human position (XY) |
| `planner_human_ori` | float | Human orientation (radians) |
| `planner_current_poses` | list[Pose] | Current EEF poses |
| `planner_rest_target` | list[Pose] | Rest mode target poses |
| `planner_approach_target` | list[Pose] | Approach mode target poses |
| `planner_last_assist_pos` | np.array | Position at last assist frame |
| `planner_last_assist_ori` | float | Orientation at last assist frame |
| `planner_last_assist_height` | float | Height at last assist frame |

## Usage

### Basic Run

```bash
ros2 run ia_robot_bt mission_bt_node
```

### With Configuration

```bash
ros2 run ia_robot_bt mission_bt_node --ros-args \
  --params-file src/ia_robot_bt/config/mission_bt_params.yaml
```

## Topics

### Subscribed
- `/goal_pose` (geometry_msgs/PoseStamped): Goal location (Nav2 also subscribes)
- `/navigate_to_pose/_action/status` (action_msgs/GoalStatusArray): Nav2 status

### Published
- `/mission_status` (std_msgs/String): Current phase (IDLE/NAVIGATION/MANIPULATION)

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `manipulation_timeout` | double | 60.0 | Max manipulation time (s) |
| `tick_rate` | double | 10.0 | Tree update frequency (Hz) |

## Behavior Nodes

### Conditions
- **IsGoalPoseReceived**: Wait for /goal_pose message
- **IsNavigationComplete**: Monitor Nav2 status for SUCCESS
- **IsManipulationComplete**: Check timeout or process termination

### Actions
- **StartManipulationAction**: Launch IK controller + mocap publisher
- **StopManipulationAction**: Terminate processes and cleanup

## Example Workflow

```bash
# Terminal 1: Start behavior tree
ros2 run ia_robot_bt mission_bt_node

# Terminal 2: Monitor status
ros2 topic echo /mission_status

# Terminal 3: Send goal (this triggers everything)
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {w: 1.0}
  }
}" --once

# Watch the sequence:
# 1. Status: IDLE → GOAL_RECEIVED
# 2. Nav2 navigates automatically
# 3. Status: NAVIGATION
# 4. Navigation completes
# 5. Status: MANIPULATION
# 6. Manipulation completes
# 7. Status: IDLE (ready for next goal)
```

## Visualization

```bash
python3 src/ia_robot_bt/scripts/visualize_tree.py
```

## Testing

```bash
cd src/ia_robot_bt
python3 -m pytest test/
```

## Troubleshooting

### "Goal received but nothing happens"
- Check if Nav2 is running: `ros2 action list | grep navigate_to_pose`
- Verify goal_pose topic: `ros2 topic echo /goal_pose`

### "Navigation status not updating"
- Check Nav2 status topic: `ros2 topic echo /navigate_to_pose/_action/status`

### "Manipulation doesn't start"
- Check Nav2 completed successfully
- View logs: `ros2 topic echo /rosout | grep BT`

## Further Reading

- **QUICKSTART.md**: Quick start guide
- **MIGRATION_GUIDE.md**: How it changed from complex version  
- **ARCHITECTURE.md**: Visual diagrams

---

**Version**: 0.2.0 (Simplified)  
**License**: MIT


## Extending the Behavior Tree

### Adding a New Action

1. Create a new class in `bt_actions.py`:
   ```python
   class MyNewAction(py_trees.behaviour.Behaviour):
       def __init__(self, name, node, blackboard):
           super().__init__(name)
           self.node = node
           self.blackboard = blackboard
       
       def update(self):
           # Your action logic here
           return py_trees.common.Status.SUCCESS
   ```

2. Add it to the tree in `mission_bt_node.py`:
   ```python
   my_action = MyNewAction("My Action", self, self.blackboard)
   # Add to sequence/selector
   ```

### Adding a New Condition

1. Create in `bt_conditions.py`:
   ```python
   class MyCondition(py_trees.behaviour.Behaviour):
       def __init__(self, name, blackboard):
           super().__init__(name)
           self.blackboard = blackboard
       
       def update(self):
           # Check your condition
           if condition_met:
               return py_trees.common.Status.SUCCESS
           return py_trees.common.Status.FAILURE
   ```

### Modifying Tree Structure

Edit `create_behavior_tree()` in `mission_bt_node.py`:

```python
# Add parallel execution
parallel = Parallel(
    name="Parallel Tasks",
    policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    children=[
        task1,
        task2,
    ]
)

# Add to sequence
sequence = Sequence(
    name="My Sequence",
    children=[
        condition1,
        parallel,
        action1,
    ]
)
```

## Future Enhancements

- [ ] Add XML tree loading support
- [ ] Implement safety monitor behaviors
- [ ] Add recovery behaviors
- [ ] Create web-based tree visualizer
- [ ] Add behavior tree execution logging
- [ ] Implement dynamic tree reconfiguration

## References

- [py_trees Documentation](https://py-trees.readthedocs.io/)
- [Nav2 Behavior Trees](https://navigation.ros.org/behavior_trees/index.html)
- [Behavior Trees in Robotics](https://arxiv.org/abs/1709.00084)
