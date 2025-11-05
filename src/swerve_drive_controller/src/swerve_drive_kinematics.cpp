// Copyright 2025 ros2_control development team
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "swerve_drive_controller/swerve_drive_kinematics.hpp"
#include "rclcpp/logging.hpp"

namespace swerve_drive_controller
{

SwerveDriveKinematics::SwerveDriveKinematics(
  const std::array<std::pair<double, double>, 4> & wheel_positions,
                  double wheel_radius,
                  double max_steering_angle,
                  double min_steering_angle)
: wheel_positions_(wheel_positions),
  wheel_radius_{wheel_radius},
  max_steering_angle_{max_steering_angle},
  min_steering_angle_{min_steering_angle},
  odometry_{0.0, 0.0, 0.0}
{
}

std::array<WheelCommand, 4> SwerveDriveKinematics::compute_wheel_commands(
  double linear_velocity_x, double linear_velocity_y, double angular_velocity_z)
{
  std::array<WheelCommand, 4> wheel_commands;

  // wx = W/2, wy = L/2

  for (std::size_t i = 0; i < 4; i++)
  {
    const auto & [wx, wy] = wheel_positions_[i];

    double vx = (linear_velocity_x - angular_velocity_z * wy) / wheel_radius_;
    double vy = (linear_velocity_y + angular_velocity_z * wx) / wheel_radius_;

    wheel_commands[i].drive_velocity = std::hypot(vx, vy);
    wheel_commands[i].steering_angle = std::atan2(vy, vx);

  }

  if (!clip_steering_angle(wheel_commands))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SwerveDriveKinematics"),
      "Failed to clip steering angles to the range [%f, %f]",
      min_steering_angle_, max_steering_angle_);
  }

  return wheel_commands;
}


bool SwerveDriveKinematics::clip_steering_angle(std::array<WheelCommand, 4> & wheel_commands)
{
  for (auto & command : wheel_commands)
  {
    if (command.steering_angle > max_steering_angle_)
    {
      if (command.steering_angle - M_PI > min_steering_angle_)
      {
        command.steering_angle -= M_PI;
        command.drive_velocity = -command.drive_velocity;  // Reverse direction
      }
      else
      {
        return false;
      }
    }
    else if (command.steering_angle < min_steering_angle_)
    {
      if (command.steering_angle + M_PI < max_steering_angle_)
      {
        command.steering_angle += M_PI;
        command.drive_velocity = -command.drive_velocity;  // Reverse direction
      }
      else
      {
        return false;
      }
    }
  }

  return true;
}



/**
 * @brief 根据轮速和转向角更新机器人里程计
 * 
 * 使用航位推算法（Dead Reckoning）计算机器人在世界坐标系中的位置和朝向
 * 基于四轮全向移动机器人的运动学模型
 * 
 * @param wheel_velocities 4个轮子的角速度（rad/s）
 * @param steering_angles 4个轮子的转向角（rad）
 * @param dt 时间步长（秒）
 * @return 更新后的里程计状态（位置x,y和朝向角theta）
 */
OdometryState SwerveDriveKinematics::update_odometry(
  const std::array<double, 4> & wheel_velocities, const std::array<double, 4> & steering_angles,
  double dt)
{
  // 步骤1: 计算机器人本体系下的速度（假设完美的轮子控制）
  // 通过4个轮子的速度合成机器人的整体运动状态
  double vx_sum = 0.0, vy_sum = 0.0, wz_sum = 0.0;
  
  // 遍历4个轮子，计算每个轮子对机器人运动的贡献
  for (std::size_t i = 0; i < 4; i++)
  {
    // 将轮子的角速度转换为线速度，并根据转向角分解到x和y方向
    // vx = ω * r * cos(δ)  - 轮子x方向的线速度分量
    // vy = ω * r * sin(δ)  - 轮子y方向的线速度分量
    double vx = wheel_velocities[i] * std::cos(steering_angles[i]) * wheel_radius_;
    double vy = wheel_velocities[i] * std::sin(steering_angles[i]) * wheel_radius_;

    // 累加所有轮子对线性速度的贡献（后续取平均）
    vx_sum += vx;
    vy_sum += vy;

    // 计算每个轮子对机器人角速度的贡献
    // 使用叉积原理：r × v，其中r是轮子相对于机器人中心的位置向量
    // wz_sum += (vy * wx - vx * wy)，wx和wy是轮子位置的x和y坐标
    wz_sum += (vy * wheel_positions_[i].first - vx * wheel_positions_[i].second);
  }

  // 步骤2: 计算机器人整体的平均线速度（4个轮子的平均值）
  double vx_robot = vx_sum / 4.0;
  double vy_robot = vy_sum / 4.0;

  // 步骤3: 计算角速度的分母部分（轮子位置到中心距离的平方和）
  // 用于将轮子产生的角动量转换为机器人的角速度
  double wz_denominator = 0.0;
  for (std::size_t i = 0; i < 4; i++)
  {
    wz_denominator +=
      (wheel_positions_[i].first * wheel_positions_[i].first +
       wheel_positions_[i].second * wheel_positions_[i].second);
  }
  
  // 计算机器人的角速度（rad/s）
  double wz_robot = wz_sum / wz_denominator;

  // 步骤4: 将机器人本体系的速度转换到世界坐标系
  // 使用旋转矩阵进行坐标变换
  double cos_theta = std::cos(odometry_.theta);
  double sin_theta = std::sin(odometry_.theta);

  // 机器人本体系 -> 世界坐标系 的速度变换
  // [vx_global]   [cos(θ)  -sin(θ)]   [vx_robot]
  // [vy_global] = [sin(θ)   cos(θ)] * [vy_robot]
  double vx_global = vx_robot * cos_theta - vy_robot * sin_theta;
  double vy_global = vx_robot * sin_theta + vy_robot * cos_theta;

  // 步骤5: 积分计算新的位置和朝向
  // 位置更新：当前位置 + 速度 × 时间
  odometry_.x += vx_global * dt;
  odometry_.y += vy_global * dt;
  
  // 朝向角更新，并使用normalize_angle确保角度在[-π, π]范围内
  odometry_.theta = normalize_angle(odometry_.theta + wz_robot * dt);
  
  return odometry_;
}

double SwerveDriveKinematics::normalize_angle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

void SwerveDriveKinematics::update_parameters(
  const std::array<std::pair<double, double>, 4> & wheel_positions,
  double wheel_radius,
  double max_steering_angle,
  double min_steering_angle)
{
  wheel_positions_ = wheel_positions;
  wheel_radius_ = wheel_radius;
  max_steering_angle_ = max_steering_angle;
  min_steering_angle_ = min_steering_angle;
}

}  // namespace swerve_drive_controller
