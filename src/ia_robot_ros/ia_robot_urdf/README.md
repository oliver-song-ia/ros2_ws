# IA Robot URDF Package

> **⚠️ INTUITIVE AUTONOMY CONFIDENTIAL ⚠️**
> 
> This package and all its contents are proprietary and confidential to Intuitive Autonomy.
> All files, documentation, designs, and technical specifications contained within this
> package are the exclusive property of Intuitive Autonomy and are protected by applicable
> intellectual property laws. Unauthorized use, reproduction, distribution, or disclosure
> is strictly prohibited.

This package contains the URDF (Unified Robot Description Format) files and associated resources for the IA Robot. The package provides multiple URDF variants for different use cases and includes all necessary 3D models, launch files, and configuration.

## Package Contents

### URDF Files

The package includes three main URDF files located in the `urdf/` directory:

#### 1. `ia_robot.urdf` (Original URDF)
- Uses relative paths with `package://` URI scheme for mesh files

#### 2. `ia_robot.absolute.urdf` (Absolute Path Version)
- Replaces `package://` URIs with absolute filesystem paths
- Useful for applications that don't support ROS package URIs (e.g. isaac sim)
- **Please update the paths** before you use this file

#### 3. `ia_robot_ik.urdf` (Inverse Kinematics Version)
- Includes 3 virtual joints for chassis movement during IK calculations: these virtual joints allow the IK solver to consider the robot's base mobility when planning arm movements, enabling coordinated whole-body motion.
  - `chassis_x_joint`: Prismatic joint for X-axis translation
  - `chassis_y_joint`: Prismatic joint for Y-axis translation
  - `chassis_rotation_joint`: Continuous joint for Z-axis rotation


## Usage

### Visualization
```bash
# Launch robot visualization with interactive joint controls
ros2 launch ia_robot_urdf display.launch.py
```

### Updating Absolute Path URDF
```bash
# Run the conversion script
python3 ia_robot_urdf/scripts/convert_urdf_to_absolute.py
```

This will regenerate `ia_robot.absolute.urdf` with updated absolute paths.
