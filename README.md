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