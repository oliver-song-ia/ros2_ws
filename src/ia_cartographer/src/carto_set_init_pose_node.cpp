#include "rclcpp/rclcpp.hpp"
#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <stdexcept>

using namespace std::chrono_literals;
using FinishTrajectory = cartographer_ros_msgs::srv::FinishTrajectory;
using StartTrajectory = cartographer_ros_msgs::srv::StartTrajectory;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

class CartoSetInitialPoseNode : public rclcpp::Node {
public:
    CartoSetInitialPoseNode() : Node("carto_set_initialpose_node"), traj_id_(1) {
        // 创建初始姿态订阅者
        pose_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
            "/initialpose", 
            100,  // QoS深度
            std::bind(&CartoSetInitialPoseNode::init_pose_callback, this, std::placeholders::_1));

        // 创建服务客户端
        finish_client_ = this->create_client<FinishTrajectory>("finish_trajectory");
        start_client_ = this->create_client<StartTrajectory>("start_trajectory");

        // 等待服务可用
        if (!finish_client_->wait_for_service(5s)) {
            RCLCPP_WARN(this->get_logger(), "Finish trajectory service not available within 5s");
        }
        if (!start_client_->wait_for_service(5s)) {
            RCLCPP_WARN(this->get_logger(), "Start trajectory service not available within 5s");
        }

        RCLCPP_INFO(this->get_logger(), "ROS2 Carto initial pose node initialized");
    }

private:
    void init_pose_callback(const PoseWithCovarianceStamped::SharedPtr pose_data) {
        // 解析位姿数据
        double poseX = pose_data->pose.pose.position.x;
        double poseY = pose_data->pose.pose.position.y;
        
        // 计算Yaw角
        tf2::Quaternion quat(
            pose_data->pose.pose.orientation.x,
            pose_data->pose.pose.orientation.y,
            pose_data->pose.pose.orientation.z,
            pose_data->pose.pose.orientation.w);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "Received initial pose - x:%.3f, y:%.3f, yaw:%.3f", poseX, poseY, yaw);

        // 异步调用结束轨迹服务
        auto finish_req = std::make_shared<FinishTrajectory::Request>();
        finish_req->trajectory_id = traj_id_;
        
        if (!finish_client_->service_is_ready()) {
            RCLCPP_ERROR(this->get_logger(), "Finish trajectory service not ready!");
            return;
        }

        auto finish_future = finish_client_->async_send_request(
            finish_req,
            [this, pose_data](rclcpp::Client<FinishTrajectory>::SharedFuture future) {
                try {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "Successfully finished trajectory %d", this->traj_id_);
                    this->start_new_trajectory(pose_data);  // 结束成功后启动新轨迹
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to finish trajectory: %s", e.what());
                }
            }
        );
    }

    void start_new_trajectory(const PoseWithCovarianceStamped::SharedPtr pose_data) {
        auto start_req = std::make_shared<StartTrajectory::Request>();
        start_req->configuration_directory = "/home/ia/fangfudong/ros2_ws/src/ia_cartographer/config";
        start_req->configuration_basename = "ia_location.lua";
        start_req->use_initial_pose = true;
        start_req->initial_pose = pose_data->pose.pose;
        start_req->relative_to_trajectory_id = 0;

        if (!start_client_->service_is_ready()) {
            RCLCPP_ERROR(this->get_logger(), "Start trajectory service not ready!");
            return;
        }

        auto start_future = start_client_->async_send_request(
            start_req,
            [this](rclcpp::Client<StartTrajectory>::SharedFuture future) {
                try {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "Successfully started new trajectory %d", this->traj_id_ + 1);
                    this->traj_id_++;  // 仅在启动成功时递增轨迹ID
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to start trajectory: %s", e.what());
                }
            }
        );
    }

private:
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Client<FinishTrajectory>::SharedPtr finish_client_;
    rclcpp::Client<StartTrajectory>::SharedPtr start_client_;
    int traj_id_;  // 轨迹ID计数器
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartoSetInitialPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
