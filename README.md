## 启动 ia-bot 001
```
conda deactivate
source install/setup.bash
ros2 launch turn_on_wheeltec_robot wheeltec_sensors.launch.py
```

## 启动键盘控制
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 订阅odom
```
ros2 topic echo /swerve_drive_controller/odom --field pose.pose
```

## 可视化odom
```
conda activate human-pose
python visualize_odom.py
```



## nav2 改动
1. `/ia_slam_toolbox/launch/navigation_launch.py`:
   - Modified the remapping of `cmd_vel` and `cmd_vel_smoothed` topics to integrate with the ultrasonic safety system.
   - The `cmd_vel` from Nav2 is now remapped to `cmd_vel_controller`, and the smoothed velocity commands are published to `cmd_vel_nav`.