# Mobile Robot Obstacle Avoidance with Inverse Kinematics

## Overview

This implementation adds environment obstacle avoidance to the existing pinocchio-based inverse kinematics system. The system converts point cloud sensor data to a 3D Signed Distance Field (SDF) and integrates obstacle distance constraints into the IK optimization objective.

## System Architecture

### Core Components

1. **SignedDistanceField** (`signed_distance_field.py`)
   - Converts point cloud data to continuous 3D distance field
   - Uses efficient distance transform algorithms
   - Provides trilinear interpolation for smooth distance queries
   - Supports gradient computation for optimization

2. **CollisionChecker** (`collision_checker.py`)
   - Computes distances between robot links and obstacles
   - Uses simplified geometric representations (spheres, cylinders, boxes)
   - Provides collision cost functions for optimization
   - Supports configurable safety margins

3. **MobileIKConfig** (`mobile_ik_config.py`)
   - Centralized configuration for all parameters
   - Preset configurations for different scenarios
   - Tunable cost function parameters
   - Performance vs. accuracy trade-offs

4. **Enhanced SimpleMobileIK** (`simple_mobile_ik.py`)
   - Modified objective function with obstacle avoidance term
   - Configurable weights for balancing task completion vs. safety
   - Integrated collision checking in optimization loop

5. **Enhanced MobileIKController** (`mobile_ik_controller.py`)
   - Point cloud subscriber for real-time obstacle data
   - Rate-limited SDF updates for performance
   - Comprehensive visualization publishers
   - Real-time collision monitoring

## Features

### Obstacle Detection
- **Point Cloud Processing**: Subscribes to `/camera/depth/points` (sensor_msgs/PointCloud2)
- **Filtering**: Removes invalid points (NaN, out of range)
- **Downsampling**: Configurable point cloud size limits for performance
- **Rate Limiting**: SDF updates every N messages to balance accuracy vs. performance

### Signed Distance Field
- **3D Grid Representation**: Efficient spatial data structure
- **Fast Distance Transform**: O(n) algorithm for distance computation  
- **Trilinear Interpolation**: Smooth distance queries at arbitrary points
- **Gradient Computation**: Enables gradient-based optimization
- **Configurable Resolution**: Trade-off between accuracy and memory usage

### Collision Checking
- **Multi-Link Checking**: Monitors all important robot links
- **Geometric Approximation**: Spheres/cylinders/boxes for efficiency
- **Safety Margins**: Configurable safety buffer around robot
- **Exponential Cost Function**: Strong penalty near obstacles, gentle far away

### Optimization Integration
- **Weighted Objective**: Balances task completion with obstacle avoidance
- **Configurable Weights**: Tune for different scenarios
- **Smooth Cost Function**: Differentiable for gradient-based optimization
- **Real-time Performance**: Fast enough for control loop integration

### Visualization & Monitoring
- **SDF Visualization**: RViz markers showing distance field
- **Collision Distances**: Real-time distance monitoring
- **Processed Obstacles**: Filtered point cloud visualization
- **Performance Monitoring**: Timing and cost metrics

## Configuration Options

### Preset Configurations

1. **High Precision Mode**
   - SDF Resolution: 0.05m
   - Update Rate: Every 2 messages
   - Higher accuracy, slower performance

2. **Performance Mode**
   - SDF Resolution: 0.12m  
   - Update Rate: Every 8 messages
   - Faster performance, good accuracy

3. **Aggressive Avoidance**
   - High obstacle weights (25.0)
   - Large safety margins (0.25m)
   - Conservative behavior

4. **Conservative Avoidance**
   - Low obstacle weights (3.0)
   - Small safety margins (0.1m)
   - Prioritizes task completion

### Key Parameters

```python
# SDF Configuration
SDF_RESOLUTION = 0.08  # Grid resolution (meters)
SDF_BOUNDS = ([-3,-3,-0.5], [3,3,3])  # Workspace bounds
SDF_MAX_DISTANCE = 2.0  # Maximum computed distance

# Collision Parameters
COLLISION_MARGIN = 0.15  # Safety margin (meters)
OBSTACLE_AVOIDANCE_WEIGHT = 10.0  # Optimization weight

# Performance Parameters
POINTCLOUD_DOWNSAMPLE_SIZE = 3000  # Max points processed
SDF_UPDATE_RATE = 5  # Update every N messages
```

## Usage

### Basic Usage

1. **Start the Controller**:
   ```bash
   ros2 run ia_robot kinematics/mobile_ik_controller.py
   ```

2. **Provide Point Cloud Data**:
   ```bash
   # Publish to /camera/depth/points (sensor_msgs/PointCloud2)
   ```

3. **Send Target Poses**:
   ```bash
   # Publish to /demo_target_poses (geometry_msgs/PoseArray)
   # Format: [left_arm_target, right_arm_target]
   ```

### Monitoring & Debugging

- **Collision Distances**: `/collision_distances` (std_msgs/Float32MultiArray)
- **SDF Visualization**: `/sdf_visualization` (visualization_msgs/MarkerArray)  
- **Processed Obstacles**: `/processed_obstacles` (sensor_msgs/PointCloud2)
- **Joint States**: `/joint_states` (sensor_msgs/JointState)

### Configuration Tuning

```python
# Load preset configuration
from kinematics.mobile_ik_config import PresetConfigs
PresetConfigs.aggressive_avoidance()

# Or customize parameters
from kinematics.mobile_ik_config import MobileIKConfig
MobileIKConfig.OBSTACLE_AVOIDANCE_WEIGHT = 15.0
MobileIKConfig.COLLISION_MARGIN = 0.2
```

## Testing

Run the comprehensive test suite:

```bash
cd /home/hanxi/code/ia_robot_sim/src/ia_robot/src
python3 test_obstacle_avoidance.py
```

The test verifies:
- ✓ SDF creation and distance queries
- ✓ Collision checking with robot geometry
- ✓ IK optimization with obstacle avoidance
- ✓ Cost reduction compared to no avoidance

## Performance Considerations

### Computational Complexity
- **SDF Creation**: O(n) where n = grid size
- **Distance Queries**: O(1) with trilinear interpolation
- **Collision Checking**: O(k) where k = number of robot links
- **Overall**: Suitable for real-time control (20 Hz)

### Memory Usage
- **SDF Grid**: ~1-10 MB depending on resolution
- **Point Cloud**: ~100 KB - 1 MB per frame
- **Total**: Reasonable for embedded systems

### Optimization Tips
1. **Reduce SDF Resolution**: Lower resolution for faster performance
2. **Increase Update Rate**: Less frequent SDF updates
3. **Downsample Point Clouds**: Fewer points for processing
4. **Adjust Workspace Bounds**: Smaller workspace = less memory

## Limitations & Future Work

### Current Limitations
1. **Static Obstacles**: Assumes obstacles don't move during optimization
2. **Geometric Approximation**: Simplified robot link geometry
3. **Local Minima**: May get stuck in local optima with complex obstacles
4. **Sensor Dependency**: Requires dense 3D sensor data

### Future Enhancements
1. **Dynamic Obstacles**: Prediction and temporal reasoning
2. **Detailed Robot Mesh**: More accurate collision geometry
3. **Multiple Sensor Fusion**: Combine different sensor modalities
4. **Learning-Based Optimization**: Neural network guided search
5. **Workspace Planning**: Global path planning integration

## Integration Notes

### Dependencies
- ROS 2 (geometry_msgs, sensor_msgs, visualization_msgs)
- NumPy (numerical computations)
- SciPy (optimization, distance transforms)
- Pinocchio (robot kinematics)
- scikit-learn (KD-tree for distance queries)

### File Structure
```
src/ia_robot/src/
├── kinematics/
│   ├── __init__.py
│   ├── mobile_ik_controller.py          # Main ROS node
│   ├── simple_mobile_ik.py             # IK solver with obstacle avoidance
│   ├── mobile_ik_config.py    # Configuration and presets
│   ├── signed_distance_field.py        # 3D SDF implementation
│   └── collision_checker.py            # Robot-obstacle collision checking
└── test_obstacle_avoidance.py      # Comprehensive test suite
```

The system is designed to be modular, configurable, and easily integrated with existing robotic systems while providing real-time obstacle avoidance capabilities for complex mobile manipulators.
