# IA Robot ROS

This repository contains the core algorithms and ROS packages for the IA Robot platform.

## Packages

- **ia_robot** - Main package with launch files and configurations
- **ia_robot_bt** - Behavior tree implementation for mission coordination
- **ia_robot_interfaces** - Custom ROS message and action definitions
- **ia_robot_navigation** - [TODO] Navigation stack (based on Nav2) for path planning and obstacle avoidance
- **ia_robot_perception** - Perception modules including human tracking and SPIKE segmentation
- **ia_robot_planner** - High-level motion planner with trajectory optimization
- **ia_robot_urdf** - Robot description files and models
- **robot_self_filter** - Point cloud self-filtering for robot body occlusion
- **swerve_drive_controller** - Swerve drive kinematics and control

# Setup Instructions

## 1. Navigation Stack
Follow the instructions in [Integration Guide for Navigation](./doc/integration_guide_navigation_zh.md) to set up the navigation stack.

### Note:

The nav2 code is currently removed from this repo to avoid conflicts with those in ros2_ws.

Consider moving the navigation code to a new repo `ia_robot_navigation` and add that as a submodule here.

!! Apply the following changes to make twist_mux work:

1. `/nav2_bringup/launch/navigation_launch.py`:
    - Refer to this commit: https://github.com/Intuitive-Autonomy/ia_robot_ros/commit/f98cbb64962aadb432fce51980731dca9e06af00
    - Modified the remapping of `cmd_vel` and `cmd_vel_smoothed` topics to integrate with the ultrasonic safety system.
    - The `cmd_vel` from Nav2 is now remapped to `cmd_vel_controller`, and the smoothed velocity commands are published to `cmd_vel_nav`.


## 2. Download Human Tracking Model
Download the pretrained human tracking models from the links below and put the model in `/ia_robot_perception/human_tracking/saves/stcn.pth`, or use `download_model.py`.

s012 model (better): [[Google Drive]](https://drive.google.com/file/d/1mRrE0uCI2ktdWlUgapJI_KmgeIiF2eOm/view?usp=sharing) [[OneDrive]](https://uillinoisedu-my.sharepoint.com/:u:/g/personal/hokeikc2_illinois_edu/Eav35v3GZIZFiq6dv9BM8n0BHtR1hD7QU9tcxH7hylG3dA?e=NXJpTu)


## 3. Set up Spike Environment

Follow the instructions in [Spike README](./ia_robot_perception/spike/README.md) to set up the Spike environment.

Download the pretrained Spike model from [[Google Drive]](https://drive.google.com/file/d/10vkQWkEbqKAlv_-5XhsdqgOSJPQ52tjf/view?usp=drive_link) and place it in `ia_robot_perception/spike/experiments/Custom/pretrained-full/spike_oliver_1022.pth`.

## 4. Sit2Stand Task

Refer to [This section](https://github.com/Intuitive-Autonomy/ia_robot_sim#perception--planning-standalone)