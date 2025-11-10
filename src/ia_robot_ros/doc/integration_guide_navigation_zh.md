

# 控制/导航启动指南

> **概述**：本指南介绍如何启动机器人的控制和导航系统。主要包括三个步骤：
> 1. 环境准备与构建（步骤1-2）
> 2. 启动基础模块（步骤3-4）：模拟器/真实机器人 + ROS工具链
> 3. 导航功能（步骤5-6）：建图（可选）+ 自主导航

## 1. 把本仓库作为子模块添加到工作区
如果工作区中已有本仓库包含的模块，需要删除现有模块

```bash
# 在ros工作区根目录执行
git submodule add git@github.com:Intuitive-Autonomy/ia_robot_ros.git src/ia_robot_ros
```

## 2. 构建

```bash
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --parallel-workers 2 # 限制并行工作进程以避免内存问题
source ./install/setup.bash
```

## 3. **[供参考]** 启动模拟器

**注意**：
- 如果使用**真实机器人**，跳过此步骤，确保真实机器人发布 `/joint_states` 和 `/lidar` 等传感器话题即可
- 如果使用**仿真环境**，运行以下命令启动 Genesis 模拟器
```bash
# 在ia_robot_sim模拟器仓库工作区根目录执行
python ia_robot_sim/src/genesis_ros/test_import.py
```

**模拟器发布的话题**：
- `/joint_states`：关节状态（位置、速度）
- `/lidar`：激光雷达点云
- `/odom`：里程计（仅作为 ground truth 参考）

**真实机器人与模拟器的区别**：
- 真实机器人需要下位机发布 `/joint_states` 和传感器数据
- 里程计 `/odom` 将由控制器模块根据 `/joint_states` 中的舵机和旋转轮状态计算得出（见步骤4.1）

## 4. 启动 ROS 工具链

**一键启动所有基础模块**：
```bash
ros2 launch ia_robot gs_util.launch.py
```
**该命令会启动以下5个核心模块**：

### 4.1 统一控制器 (`ctrl_launch_file`)

**功能**：协调上半身（机械臂）和下半身（底盘）的控制命令

**启动文件**：[ctrl_ia_robot.launch.py](../ia_robot/launch/ctrl_ia_robot.launch.py)
    
| 类型 | 话题名称 | 消息类型 | 说明 |
|------|---------|---------|------|
| **订阅** | `/joint_states` | JointState | 下位机发出的关节状态（位置 rad，速度 rad/s） |
| **订阅** | `/cmd_vel` | Twist | 底盘速度命令（m/s）。来源：IK模块/键盘/nav2，经twist_mux选择 |
| **订阅** | `/upper_body_controller/commands` | Float64MultiArray | 上半身关节位置命令（rad），由IK模块发出 |
| **发布** | `/joint_commands` | JointState | 上下半身所有关节的统一控制命令（位置 rad，速度 rad/s） |
| **发布** | `/swerve_drive_controller/odom` | Odometry | 根据舵机和旋转轮状态计算的里程计 |

### 4.2 机器人状态发布器 (`robot_state_publisher`)

**功能**：读取URDF模型，将关节状态转换为TF坐标变换树
- **订阅**：`/joint_states`
- **发布**：`/tf` (坐标变换关系)

### 4.3 可视化工具 (`rviz2`)

**功能**：3D可视化界面，显示机器人模型、传感器数据、导航路径等

### 4.4 机器人本体点云过滤器 (`self_filter`)

**功能**：从激光雷达点云中移除机器人本体的点，避免将自身识别为障碍物
**启动时间**：约30秒处理URDF网格，请耐心等待

**运行性能**：处理完成后，每帧（~5000点）耗时 <10ms
| 类型 | 话题名称 | 说明 |
|------|---------|------|
| **订阅** | `/lidar`, `/pointcloud_human_only` | 原始点云数据 |
| **订阅** | `/tf` | 用于获取机器人各link的当前位置 |
| **发布** | `/lidar_filter`, `/pointcloud_human_only_filter` | 过滤后的点云（移除了机器人本体） |
### 4.5 点云转激光扫描 (`Pointcloud_to_laserscan`)

**功能**：将3D点云转换为2D激光扫描数据，供导航模块使用
- **订阅**：`/lidar_filter` （已过滤机器人本体的点云）
- **发布**：`/scan` （2D激光扫描数据）


## 5. **[可选]** 遥控机器人构建地图（SLAM）

> **何时需要此步骤**：如果已经用 cartographer 或其他工具保存好了地图，可以**跳过此步骤**，直接进入步骤6
### 5.1 启动 SLAM + Nav2
```bash
# 设置环境变量，避免 Ctrl+C 后进程挂起
export LAUNCH_SIGTERM_TIMEOUT=2
export LAUNCH_SIGKILL_TIMEOUT=2

# 启动带SLAM功能的导航系统
ros2 launch ia_robot nav2_with_alignment.launch.py slam:=True
```
### 5.2 遥控机器人建图
**方式1：键盘遥控（推荐用于熟悉环境）**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
**方式2：自主探索（可选）**
```bash
ros2 launch explore_lite explore.launch.py 
```
### 5.3 保存地图
当您对地图满意后，执行以下命令保存。地图将保存为两个文件：
- `<name>.pgm`：地图图像
- `<name>.yaml`：地图元数据（分辨率、原点等）
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '~/code/ia_robot_sim/src/ia_robot_ros/ia_robot/maps/map_1020'}}"
```

## 6. 使用预构建地图进行自主导航

### 6.1 启动的模块
[启动文件](../ia_robot/launch/nav2_with_alignment.launch.py)会启动以下模块：
1. **Nav2 导航系统**：路径规划、局部避障、行为树控制（默认不开启SLAM）

2. **安全系统** (详见[此文档](https://github.com/Intuitive-Autonomy/ia_robot_sim/blob/main/doc/ULTRASONIC_SAFETY_SYSTEM.md))：
   - `safety_system.launch.py`：超声波传感器急停功能
   - `twist_mux`：根据优先级选择不同来源的速度命令并发送到 `/cmd_vel`

### 6.2 启动导航系统
**前置条件**：确保已完成以下设置（参考后续"参数设置指南"章节）
- ✅ 地图路径配置
- ✅ Nav2 参数文件配置
- ✅ 机器人尺寸参数（footprint、轮子参数等）
```bash
export LAUNCH_SIGTERM_TIMEOUT=2
export LAUNCH_SIGKILL_TIMEOUT=2
ros2 launch ia_robot nav2_with_alignment.launch.py
```
### 6.3 在 RViz2 中发送导航目标
1. 在 RViz2 界面中，将**参考坐标系**（Fixed Frame）改为 `map`
2. 使用工具栏中的 **"2D Goal Pose"** 工具
3. 在地图上点击并拖动，设置目标位置和朝向
4. 机器人将自动规划路径并导航到目标点
**调试提示**：
- 在 RViz2 中可以查看 `/global_plan`（全局路径）和 `/local_plan`（局部路径）
- 观察 `/local_costmap` 和 `/global_costmap` 确认障碍物检测是否正常



# 参数设置指南

> **重要提示**：在首次运行或更换机器人硬件后，务必检查并调整以下参数

## 一、控制器参数

### 1.1 启动文件检查

**文件位置**：[ctrl_ia_robot.launch.py](../ia_robot/launch/ctrl_ia_robot.launch.py)

**检查项**：
- ✅ 确认 URDF 文件是否为[最新版本](https://github.com/Intuitive-Autonomy/ia_robot_urdf/blob/main/urdf/ia_robot.urdf)
- ✅ URDF 中的关节名称、link名称是否与实际机器人匹配

### 1.2 控制器配置文件

**文件位置**：[ia_robot_controller.yaml](../ia_robot/config/ia_robot_controller.yaml#L35)

#### 1.2.1 轮子几何参数

这些参数直接影响运动学计算的准确性，**必须根据实际机器人测量值设置**：

| 参数名称 | 说明 | 单位 | 默认值 |
|---------|------|------|--------|
| `wheel_x_offset` | 轮轴中心线到底盘中心的纵向距离 | 米 | 0.5 |
| `wheel_y_offset` | 轮子中心到机器人纵向中线的横向距离 | 米 | 0.25227 |
| `wheel_radius` | 轮子半径 | 米 | 0.085 |
| `max_steering_angle` | 舵轮最大转向角度 | 弧度 | 1.5708 (π/2) |
| `min_steering_angle` | 舵轮最小转向角度 | 弧度 | -1.5708 (-π/2) |

**如何测量**：
```
wheel_x_offset: 从底盘中心到前/后轮轴的距离
wheel_y_offset: 从机器人中线到左/右轮中心的距离
wheel_radius: 轮子的实际半径（可以用卷尺测量直径后除以2）
```

#### 1.2.2 速度与转向控制参数

| 参数名称 | 说明 | 单位 | 默认值 | 调整建议 |
|---------|------|------|--------|----------|
| `velocity_threshold` | 轮子最大角速度限制 | rad/s | 50 | 根据电机性能调整 |
| `steering_angle_tolerance` | 可接受的转向角误差阈值 | 弧度 | 0.1 | 超过此值时降低速度防止打滑 |
| `max_steering_error_for_zero_velocity` | 转向角误差的停止阈值 | 弧度 | 0.5 | 超过此值时车轮速度降为零 |

**工作原理**：
- 当 `实际转向角误差 > steering_angle_tolerance` 时，车轮速度开始降低
- 当 `实际转向角误差 > max_steering_error_for_zero_velocity` 时，车轮完全停止
- 这种机制防止舵轮未转到位时车轮就开始转动，避免轮胎侧向打滑


## 二、导航参数


### 2.1 导航启动文件配置

**文件位置**：[nav2_with_alignment.launch.py](../ia_robot/launch/nav2_with_alignment.launch.py)

| 参数名称 | 说明 | 示例值 | 备注 |
|---------|------|--------|------|
| `map` | 预构建地图文件路径 | `map_1020.yaml` | 由 SLAM toolbox/cartographer 生成，传入 `.yaml` 格式 |
| `params_file` | Nav2 配置文件 | `nav2_params_genesis.yaml` | 详见 2.2 节 |
| `alignment_points_file` | 狭窄区域引导点 | `map_0722_empty.json`（空） | 需要根据地图单独配置，参数调好后通常不需要 |
| `behavior_tree_file` | 导航行为树 | `navigate_backup_first.xml` | 定义遇到障碍时的恢复策略，见 2.3 节 |
| `use_sim_time` | 是否使用仿真时间 | `true`/`false` | `true`：使用 `/clock` 话题的时间戳（仿真环境）<br>`false`：使用系统时间（真实机器人） |

### 2.2 Nav2 核心配置文件

**文件位置**：[nav2_params_genesis.yaml](../ia_robot/config/nav2_params_genesis.yaml)

该文件包含了所有 Nav2 模块的详细参数，参数都有注释说明。以下是**较为重要**的关键参数：

#### 2.2.1 机器人尺寸（Footprint）

机器人的碰撞边界（米），用于避障计算：

```yaml
# 在两处设置（local_costmap 和 global_costmap）
local_costmap:
  local_costmap:
    footprint: "[ [0.55, 0.35], [0.55, -0.35], [-0.55, -0.35], [-0.55, 0.35] ]"
    
global_costmap:
  global_costmap:
    footprint: "[ [0.6, 0.35], [0.6, -0.35], [-0.6, -0.35], [-0.6, 0.35] ]"
```

#### 2.2.2 障碍物膨胀层（Inflation Layer）

根据机器人尺寸和实际测试中的观察设置合适的 inflation_layer 参数：
- global_costmap 的膨胀半径（inflation radius）需要大一些，让 global path 在经过狭窄通道时，尽量靠近中间。这样就不用 alignment points 了。
- local_costmap 的膨胀半径可以小一些，local planner 会通过 footprint（机器人碰撞边界）避障。小膨胀半径让它在狭窄通道探索时有更多自由

#### 2.2.3 动态障碍物更新（可选）

如果**不希望**实时检测到的动态障碍物更新全局地图（例如：只想使用SLAM预先建好的静态地图）：

```yaml
# 在 global_costmap 配置中注释掉“obstacle_layer"障碍物层，全局地图将不会根据 `/scan` 数据更新障碍物
global_costmap:
  global_costmap:
    plugins: ["static_layer", "inflation_layer"]
    # plugins: ["static_layer", "obstacle_layer", "inflation_layer"]  # 注释这行
```

### 2.3 行为树详解

**文件位置**：[navigate_backup_first.xml](../ia_robot/behavior_trees/navigate_backup_first.xml)

行为树定义了机器人在导航过程中的决策逻辑，特别是遇到障碍时的恢复行为。当前行为树主要逻辑：
        
1. **主导航流程** (NavigateWithReplanning)：
   - 以 1Hz 的频率计算从当前位置到目标的全局路径
   - 使用局部规划器跟随该路径
   - 如果路径规划或跟随失败，会清除对应的代价地图并重试

2. **恢复行为序列** (RecoveryActions)：当主导航流程失败时，按以下顺序尝试恢复动作（RoundRobin节点会循环执行）：
   - **清除代价地图**：清除局部和全局代价地图中的障碍物信息，可能解决误检问题
   - **前进微调** (DriveOnHeading)：保持当前朝向前进 0.05m，速度 0.15m/s
   - **后退** (BackUp)：后退 0.05m，速度 0.15m/s（优先尝试后退而非旋转，适合狭窄空间）
   - **等待** (Wait)：原地等待 0.5 秒，让动态障碍物通过
   - **旋转** (Spin)：原地旋转 0.2 弧度，尝试新的朝向

3. **重试机制**：整个导航恢复过程最多重试 6 次 (number_of_retries="6")

这个行为树相比默认配置的特点是：**在恢复动作中优先尝试后退，而不是旋转**。这对于舵轮机器人在狭窄通道中更有效，因为旋转可能需要更大的空间。

### 2.4 其他配置说明

**控制器类型**：
- 当前使用的是基于 DWB (Dynamic Window Approach) 的局部规划器
- 如果遇到局部规划器无法绕过障碍物的问题，可以尝试调整 DWB 参数或切换到其他控制器（如 TEB）

**参考资料**：
- Nav2 官方调参指南：https://docs.nav2.org/tuning/index.html
- DWB 算法论文：https://arxiv.org/pdf/1706.09068
