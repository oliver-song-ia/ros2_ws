// ============================ self_filter.cpp ============================
#include <chrono>
#include <sstream>
#include <memory>
#include <cstring>  // for memcpy

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <gs_ros_interfaces/msg/point_cloud2_with_index.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "robot_self_filter/self_see_filter.h"
#include "robot_self_filter/point_ouster.h"
#include "robot_self_filter/point_hesai.h"
#include "robot_self_filter/point_pandar.h"
#include "robot_self_filter/point_robosense.h"

#include <yaml-cpp/yaml.h>
#include <robot_self_filter/bodies.h>
#include <robot_self_filter/shapes.h>

// PCL includes for downsampling
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

namespace robot_self_filter
{

  enum class SensorType : int
  {
    XYZSensor = 0,
    XYZRGBSensor = 1,
    OusterSensor = 2,
    HesaiSensor = 3,
    RobosenseSensor = 4,
    PandarSensor = 5,
  };

  class SelfFilterNode : public rclcpp::Node
  {
  public:
    SelfFilterNode()
        : Node("self_filter")
    {
      try
      {
        this->declare_parameter<bool>("use_sim_time", true);
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
      }
      catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &)
      {
      }

      this->declare_parameter<std::string>("sensor_frame", "none"); // Default value
      // this->set_parameter(rclcpp::Parameter("sensor_frame", "Lidar")); // Removed explicit set
      this->declare_parameter<bool>("use_rgb", false);
      this->declare_parameter<int>("max_queue_size", 10);
      this->declare_parameter<int>("lidar_sensor_type", 0);
      this->declare_parameter<std::string>("robot_description", "");
      this->declare_parameter<std::string>("in_pointcloud_topic", "/points_dep");
      this->declare_parameter<bool>("use_custom_pointcloud_msg", false);
      
      // Downsampling parameters
      this->declare_parameter<bool>("enable_downsampling", true);
      this->declare_parameter<double>("voxel_leaf_size", 0.05);  // 5cm voxel size
      this->declare_parameter<int>("max_points", 5000);  // Maximum points after downsampling
      this->declare_parameter<std::string>("downsample_method", "voxel");  // "voxel" or "random"
      
      // Ground filtering parameters
      this->declare_parameter<bool>("enable_ground_filtering", true);
      this->declare_parameter<double>("ground_z_threshold", 0.03);  // Points below this z value are considered ground
      this->declare_parameter<std::string>("target_frame", "world");  // Frame to transform points to before filtering

      sensor_frame_ = this->get_parameter("sensor_frame").as_string();
      use_rgb_ = this->get_parameter("use_rgb").as_bool();
      max_queue_size_ = this->get_parameter("max_queue_size").as_int();
      int temp_sensor_type = this->get_parameter("lidar_sensor_type").as_int();
      sensor_type_ = static_cast<SensorType>(temp_sensor_type);
      in_topic_ = this->get_parameter("in_pointcloud_topic").as_string();
      use_custom_pointcloud_msg_ = this->get_parameter("use_custom_pointcloud_msg").as_bool();
      
      // Get downsampling parameters
      enable_downsampling_ = this->get_parameter("enable_downsampling").as_bool();
      voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
      max_points_ = this->get_parameter("max_points").as_int();
      downsample_method_ = this->get_parameter("downsample_method").as_string();
      
      // Get ground filtering parameters
      enable_ground_filtering_ = this->get_parameter("enable_ground_filtering").as_bool();
      ground_z_threshold_ = this->get_parameter("ground_z_threshold").as_double();
      target_frame_ = this->get_parameter("target_frame").as_string();

      RCLCPP_INFO(this->get_logger(), "Parameters:");
      RCLCPP_INFO(this->get_logger(), "  sensor_frame: %s", sensor_frame_.c_str());
      RCLCPP_INFO(this->get_logger(), "  use_rgb: %s", use_rgb_ ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "  max_queue_size: %d", max_queue_size_);
      RCLCPP_INFO(this->get_logger(), "  lidar_sensor_type: %d", temp_sensor_type);
      RCLCPP_INFO(this->get_logger(), "  in_pointcloud_topic: %s", in_topic_.c_str());
      RCLCPP_INFO(this->get_logger(), "  enable_downsampling: %s", enable_downsampling_ ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "  voxel_leaf_size: %.3f", voxel_leaf_size_);
      RCLCPP_INFO(this->get_logger(), "  max_points: %d", max_points_);
      RCLCPP_INFO(this->get_logger(), "  downsample_method: %s", downsample_method_.c_str());
      RCLCPP_INFO(this->get_logger(), "  enable_ground_filtering: %s", enable_ground_filtering_ ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "  ground_z_threshold: %.3f", ground_z_threshold_);
      RCLCPP_INFO(this->get_logger(), "  target_frame: %s", target_frame_.c_str());
      RCLCPP_INFO(this->get_logger(), "  use_custom_pointcloud_msg: %s", use_custom_pointcloud_msg_ ? "true" : "false");

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_buffer_->setCreateTimerInterface(
          std::make_shared<tf2_ros::CreateTimerROS>(
              this->get_node_base_interface(),
              this->get_node_timers_interface()));
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      if (use_custom_pointcloud_msg_)
      {
        pointCloudPublisherCustom_ =
            this->create_publisher<gs_ros_interfaces::msg::PointCloud2WithIndex>("cloud_out", 1);
        
        // Only create debug publisher if debug logging is enabled
        if (rcutils_logging_logger_is_enabled_for(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG))
        {
          pointCloudPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_debug", 1);
          RCLCPP_DEBUG(this->get_logger(), "Debug visualization publisher created");
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Debug logging disabled - skipping debug visualization publisher");
        }
      }
      else
      {
        pointCloudPublisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", 1);
      }

      marker_pub_ =
          this->create_publisher<visualization_msgs::msg::MarkerArray>("collision_shapes", 1);
    }

    void initSelfFilter()
    {
      std::string robot_description_xml = this->get_parameter("robot_description").as_string();

      switch (sensor_type_)
      {
      case SensorType::XYZSensor:
        self_filter_ = std::make_shared<filters::SelfFilter<pcl::PointXYZ>>(this->shared_from_this());
        break;
      case SensorType::XYZRGBSensor:
        self_filter_ = std::make_shared<filters::SelfFilter<pcl::PointXYZRGB>>(this->shared_from_this());
        break;
      case SensorType::OusterSensor:
        self_filter_ = std::make_shared<filters::SelfFilter<PointOuster>>(this->shared_from_this());
        break;
      case SensorType::HesaiSensor:
        self_filter_ = std::make_shared<filters::SelfFilter<PointHesai>>(this->shared_from_this());
        break;
      case SensorType::RobosenseSensor:
        self_filter_ = std::make_shared<filters::SelfFilter<PointRobosense>>(this->shared_from_this());
        break;
      case SensorType::PandarSensor:
        self_filter_ = std::make_shared<filters::SelfFilter<PointPandar>>(this->shared_from_this());
        break;
      default:
        self_filter_ = std::make_shared<filters::SelfFilter<pcl::PointXYZ>>(this->shared_from_this());
        break;
      }

      self_filter_->getLinkNames(frames_);

      if (use_custom_pointcloud_msg_)
      {
        sub_custom_ = this->create_subscription<gs_ros_interfaces::msg::PointCloud2WithIndex>(
            in_topic_,
            rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable),
            std::bind(&SelfFilterNode::cloudCallbackCustom, this, std::placeholders::_1));
      }
      else
      {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            in_topic_,
            rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable),
            std::bind(&SelfFilterNode::cloudCallback, this, std::placeholders::_1));
      }
    }

  private:
    sensor_msgs::msg::PointCloud2::SharedPtr downsamplePointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_cloud)
    {
      if (!enable_downsampling_)
      {
        // Return a copy if downsampling is disabled
        auto output_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_cloud);
        return output_cloud;
      }

      auto output_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      size_t input_points = input_cloud->width * input_cloud->height;
      
      if (downsample_method_ == "voxel")
      {
        try {
          pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
          pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
          
          pcl::fromROSMsg(*input_cloud, *pcl_cloud);
          
          pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
          voxel_filter.setInputCloud(pcl_cloud);
          voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
          voxel_filter.filter(*filtered_cloud);
          
          pcl::toROSMsg(*filtered_cloud, *output_cloud);
          output_cloud->header = input_cloud->header;
          
          RCLCPP_DEBUG(this->get_logger(), "Voxel downsampling: %zu -> %zu points", 
                      pcl_cloud->points.size(), filtered_cloud->points.size());
        }
        catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "Voxel downsampling failed (%s), falling back to random sampling", e.what());
          // Fall back to random sampling if voxel fails
          return downsampleRandomly(input_cloud, input_points);
        }
      }
      else if (downsample_method_ == "random")
      {
        return downsampleRandomly(input_cloud, input_points);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Unknown downsample method: %s. Using original cloud.", downsample_method_.c_str());
        auto copy_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_cloud);
        return copy_cloud;
      }
      
      return output_cloud;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr downsampleRandomly(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_cloud, size_t input_points)
    {
      auto output_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      
      // Random sampling - works at the message level, compatible with any point type
      size_t target_points = std::min(static_cast<size_t>(max_points_), input_points);
      double skip_ratio = static_cast<double>(input_points) / target_points;
      
      // Create output by copying header and selective points
      *output_cloud = *input_cloud;
      output_cloud->data.clear();
      output_cloud->data.reserve(target_points * input_cloud->point_step);
      
      size_t point_step = input_cloud->point_step;
      size_t output_points = 0;
      
      for (size_t i = 0; i < input_points && output_points < target_points; ++i)
      {
        if (i % static_cast<size_t>(std::max(1.0, skip_ratio)) == 0)
        {
          size_t src_offset = i * point_step;
          output_cloud->data.insert(output_cloud->data.end(),
                                   input_cloud->data.begin() + src_offset,
                                   input_cloud->data.begin() + src_offset + point_step);
          output_points++;
        }
      }
      
      output_cloud->width = static_cast<uint32_t>(output_points);
      output_cloud->height = 1;
      output_cloud->row_step = output_points * point_step;
      output_cloud->header = input_cloud->header;
      
      RCLCPP_INFO(this->get_logger(), "Random downsampling: %zu -> %zu points", 
                  input_points, output_points);
      
      return output_cloud;
    }
    
    sensor_msgs::msg::PointCloud2::SharedPtr filterGroundPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_cloud)
    {
      if (!enable_ground_filtering_)
      {
        // Return a copy if ground filtering is disabled
        auto output_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_cloud);
        return output_cloud;
      }
      
      auto output_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      size_t input_points = input_cloud->width * input_cloud->height;
      
      // Create output by copying header and filtering points based on z-coordinate
      *output_cloud = *input_cloud;
      output_cloud->data.clear();
      output_cloud->data.reserve(input_points * input_cloud->point_step);
      
      size_t point_step = input_cloud->point_step;
      size_t output_points = 0;
      
      // Find the z-coordinate offset in the point structure
      size_t z_offset = 0;
      bool z_found = false;
      for (const auto& field : input_cloud->fields)
      {
        if (field.name == "z")
        {
          z_offset = field.offset;
          z_found = true;
          break;
        }
      }
      
      if (!z_found)
      {
        RCLCPP_WARN(this->get_logger(), "No 'z' field found in point cloud, skipping ground filtering");
        auto copy_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_cloud);
        return copy_cloud;
      }
      
      for (size_t i = 0; i < input_points; ++i)
      {
        size_t src_offset = i * point_step;
        
        // Extract z coordinate (assuming float32)
        float z_value;
        std::memcpy(&z_value, &input_cloud->data[src_offset + z_offset], sizeof(float));
        
        // Keep points above the threshold (filter out ground points below)
        if (z_value > ground_z_threshold_)
        {
          output_cloud->data.insert(output_cloud->data.end(),
                                   input_cloud->data.begin() + src_offset,
                                   input_cloud->data.begin() + src_offset + point_step);
          output_points++;
        }
      }
      
      output_cloud->width = static_cast<uint32_t>(output_points);
      output_cloud->height = 1;
      output_cloud->row_step = output_points * point_step;
      output_cloud->header = input_cloud->header;
      
      size_t filtered_out = input_points - output_points;
      RCLCPP_DEBUG(this->get_logger(), "Ground filtering (threshold=%.3f): %zu -> %zu points (%zu filtered out)", 
                  ground_z_threshold_, input_points, output_points, filtered_out);
      
      return output_cloud;
    }
    
    sensor_msgs::msg::PointCloud2::SharedPtr transformPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_cloud)
    {
      // Check if transformation is needed
      if (input_cloud->header.frame_id == target_frame_)
      {
        // Already in target frame, return a copy
        auto output_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_cloud);
        return output_cloud;
      }
      
      try
      {
        // Get the latest available transform
        geometry_msgs::msg::TransformStamped latest_transform;
        try 
        {
          latest_transform = tf_buffer_->lookupTransform(target_frame_, input_cloud->header.frame_id, tf2::TimePointZero);
          
          // Check time difference between input cloud and latest transform
          double input_time = rclcpp::Time(input_cloud->header.stamp).seconds();
          double transform_time = rclcpp::Time(latest_transform.header.stamp).seconds();
          double time_diff = std::abs(input_time - transform_time);
          
          // Accept if time difference is less than 100ms
          if (time_diff > 0.5) // 500ms threshold
          {
            RCLCPP_WARN(this->get_logger(), "Transform time difference too large (%.3f ms), using original cloud", 
                        time_diff * 1000.0);
            RCLCPP_WARN(this->get_logger(), "Latest transform timestamp: %.6f", transform_time);
            RCLCPP_WARN(this->get_logger(), "Input cloud timestamp: %.6f", input_time);
            auto copy_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_cloud);
            return copy_cloud;
          }
          
          RCLCPP_DEBUG(this->get_logger(), "Using latest transform with time difference: %.3f ms", time_diff * 1000.0);
        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_WARN(this->get_logger(), "Failed to get latest transform from %s to %s: %s, using original cloud", 
                      input_cloud->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
          auto copy_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_cloud);
          return copy_cloud;
        }
        
        // Transform the point cloud using the latest transform
        auto output_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        tf2::doTransform(*input_cloud, *output_cloud, latest_transform);
        
        RCLCPP_DEBUG(this->get_logger(), "Transformed point cloud from %s to %s", 
                    input_cloud->header.frame_id.c_str(), target_frame_.c_str());
        
        return output_cloud;
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s. Using original cloud.", ex.what());
        auto copy_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_cloud);
        return copy_cloud;
      }
    }
    
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud)
    {
      // Record start time
      auto start_time = std::chrono::high_resolution_clock::now();
      
      // RCLCPP_INFO(this->get_logger(), "Received cloud message with timestamp %.6f",
      //             rclcpp::Time(cloud->header.stamp).seconds());

      // RCLCPP_INFO(this->get_logger(), "Point cloud size: width = %d, height = %d, total points = %d",
      //             cloud->width, cloud->height, cloud->width * cloud->height);

      // Downsample first
      auto downsampled_cloud = downsamplePointCloud(cloud);

      // Transform points to target frame first
      auto transformed_cloud = transformPointCloud(downsampled_cloud);

      // Filter ground points after transformation
      // TODO: remove hardcoded frames
      auto ground_filtered_cloud = transformed_cloud;
      if (target_frame_ == "world" || target_frame_ == "base_footprint"){
        ground_filtered_cloud = filterGroundPoints(transformed_cloud);
      }

      sensor_msgs::msg::PointCloud2 out2;
      int input_size = 0;
      int output_size = 0;

      switch (sensor_type_)
      {
      case SensorType::XYZSensor:
      {
        auto sf_xyz = std::dynamic_pointer_cast<filters::SelfFilter<pcl::PointXYZ>>(self_filter_);
        if (!sf_xyz)
          return;
        auto mask = sf_xyz->getSelfMaskPtr();
        publishShapesFromMask(mask, cloud->header.frame_id, cloud->header.stamp);
        break;
      }
      case SensorType::OusterSensor:
      {
        auto sf_ouster = std::dynamic_pointer_cast<filters::SelfFilter<PointOuster>>(self_filter_);
        if (!sf_ouster)
          return;
        auto mask = sf_ouster->getSelfMaskPtr();
        publishShapesFromMask(mask, cloud->header.frame_id, cloud->header.stamp);
        break;
      }
      default:
        RCLCPP_ERROR(this->get_logger(), "Sensor type not handled for shape publishing");
        return;
      }

      // Process the downsampled point cloud      
      self_filter_->fillPointCloud2(ground_filtered_cloud, sensor_frame_, out2, input_size, output_size);
      pointCloudPublisher_->publish(out2);
      
      // Record end time and calculate duration
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      
      RCLCPP_DEBUG(this->get_logger(), "Self filter processing and publish took: %ld microseconds (%.3f ms)", 
                  duration.count(), duration.count() / 1000.0);
                  
      RCLCPP_DEBUG(this->get_logger(), "Processing completed - Input: %d points, Output: %d points", 
                  input_size, output_size);

      
    }

    void cloudCallbackCustom(const gs_ros_interfaces::msg::PointCloud2WithIndex::ConstSharedPtr &cloud_with_index)
    {
      // Record start time
      auto start_time = std::chrono::high_resolution_clock::now();
      
      // Extract the PointCloud2 part from PointCloud2WithIndex
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud_with_index->pointcloud);

      // Downsample the point cloud after ground filtering
      auto downsampled_cloud = downsamplePointCloud(cloud);
      auto downsample_time = std::chrono::high_resolution_clock::now();
      auto downsample_duration = std::chrono::duration_cast<std::chrono::milliseconds>(downsample_time - start_time);

      // Transform points to target frame 
      auto transformed_cloud = transformPointCloud(downsampled_cloud);


      // auto transformed_cloud = transformPointCloud(cloud);
      auto transform_time = std::chrono::high_resolution_clock::now();
      auto transform_duration = std::chrono::duration_cast<std::chrono::milliseconds>(transform_time - downsample_time);

      // Filter ground points after transformation
      auto ground_filtered_cloud = filterGroundPoints(transformed_cloud);
      auto ground_filter_time = std::chrono::high_resolution_clock::now();
      auto ground_filter_duration = std::chrono::duration_cast<std::chrono::milliseconds>(ground_filter_time - transform_time);

      sensor_msgs::msg::PointCloud2 out2;
      int input_size = 0;
      int output_size = 0;

      switch (sensor_type_)
      {
      case SensorType::XYZSensor:
      {
        auto sf_xyz = std::dynamic_pointer_cast<filters::SelfFilter<pcl::PointXYZ>>(self_filter_);
        if (!sf_xyz)
          return;
        auto mask = sf_xyz->getSelfMaskPtr();
        publishShapesFromMask(mask, cloud->header.frame_id, cloud->header.stamp);
        break;
      }
      case SensorType::OusterSensor:
      {
        auto sf_ouster = std::dynamic_pointer_cast<filters::SelfFilter<PointOuster>>(self_filter_);
        if (!sf_ouster)
          return;
        auto mask = sf_ouster->getSelfMaskPtr();
        publishShapesFromMask(mask, cloud->header.frame_id, cloud->header.stamp);
        break;
      }
      default:
        RCLCPP_ERROR(this->get_logger(), "Sensor type not handled for shape publishing");
        return;
      }
      auto mask_time = std::chrono::high_resolution_clock::now();

      // Process the downsampled point cloud
      auto filter_start_time = std::chrono::high_resolution_clock::now();      
      self_filter_->fillPointCloud2(ground_filtered_cloud, sensor_frame_, out2, input_size, output_size);
      auto filter_time = std::chrono::high_resolution_clock::now();
      auto filter_duration = std::chrono::duration_cast<std::chrono::milliseconds>(filter_time - filter_start_time);
      
      // Convert back to custom message type with preserved frame_index
      gs_ros_interfaces::msg::PointCloud2WithIndex custom_out;
      custom_out.header = out2.header;
      custom_out.pointcloud = out2;
      custom_out.frame_index = cloud_with_index->frame_index; // Preserve original frame_index
      
      pointCloudPublisherCustom_->publish(custom_out);
      
      // Only publish debug visualization if debug publisher was created
      if (pointCloudPublisher_)
      {
        pointCloudPublisher_->publish(out2);
      }

      // Record end time and calculate total duration
      auto end_time = std::chrono::high_resolution_clock::now();
      auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

      RCLCPP_INFO(this->get_logger(), "Processing completed - Input: %d points, Output: %d points, Frame Index: %u", 
                  input_size, output_size, custom_out.frame_index);
      RCLCPP_INFO(this->get_logger(), "Downsample, transfer, ground filter, robot filter, total time: %ld, %ld, %ld, %ld, %ld ms",
                  downsample_duration.count(), transform_duration.count(), ground_filter_duration.count(), filter_duration.count(), total_duration.count());
    }

    template <typename PointT>
    void publishShapesFromMask(robot_self_filter::SelfMask<PointT> *mask, const std::string &pointcloud_frame, const builtin_interfaces::msg::Time &timestamp)
    {
      if (!mask)
        return;
      const auto &bodies = mask->getBodies();
      if (bodies.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "No bodies found in SelfMask");
        return;
      }

      visualization_msgs::msg::MarkerArray marker_array;
      marker_array.markers.reserve(bodies.size());

      std::string shapes_frame = pointcloud_frame;
      for (size_t i = 0; i < bodies.size(); ++i)
      {
        const auto &see_link = bodies[i];
        const bodies::Body *body = see_link.body;
        if (!body)
          continue;

        visualization_msgs::msg::Marker mk;
        mk.header.frame_id = shapes_frame;
        mk.header.stamp = timestamp;
        mk.ns = "self_filter_shapes";
        mk.id = static_cast<int>(i);
        mk.action = visualization_msgs::msg::Marker::ADD;
        mk.lifetime = rclcpp::Duration(0, 0);
        mk.color.a = 0.5f;
        mk.color.r = 1.0f;
        mk.color.g = 0.0f;
        mk.color.b = 0.0f;

        const tf2::Transform &tf = body->getPose();
        mk.pose.position.x = tf.getOrigin().x();
        mk.pose.position.y = tf.getOrigin().y();
        mk.pose.position.z = tf.getOrigin().z();
        tf2::Quaternion q = tf.getRotation();
        mk.pose.orientation.x = q.x();
        mk.pose.orientation.y = q.y();
        mk.pose.orientation.z = q.z();
        mk.pose.orientation.w = q.w();

        switch (body->getType())
        {
        case shapes::SPHERE:
        {
          auto sphere_body = dynamic_cast<const robot_self_filter::bodies::Sphere *>(body);
          if (sphere_body)
          {
            mk.type = visualization_msgs::msg::Marker::SPHERE;
            float d = static_cast<float>(2.0 * sphere_body->getScaledRadius());
            mk.scale.x = d;
            mk.scale.y = d;
            mk.scale.z = d;
          }
          break;
        }
        case shapes::BOX:
        {
          auto box_body = dynamic_cast<const robot_self_filter::bodies::Box *>(body);
          if (box_body)
          {
            mk.type = visualization_msgs::msg::Marker::CUBE;
            float sx = static_cast<float>(2.0 * box_body->getScaledHalfLength());
            float sy = static_cast<float>(2.0 * box_body->getScaledHalfWidth());
            float sz = static_cast<float>(2.0 * box_body->getScaledHalfHeight());
            mk.scale.x = sx;
            mk.scale.y = sy;
            mk.scale.z = sz;
          }
          break;
        }
        case shapes::CYLINDER:
        {
          auto cyl_body = dynamic_cast<const robot_self_filter::bodies::Cylinder *>(body);
          if (cyl_body)
          {
            mk.type = visualization_msgs::msg::Marker::CYLINDER;
            float radius = static_cast<float>(cyl_body->getScaledRadius());
            float length = static_cast<float>(2.0 * cyl_body->getScaledHalfLength());
            mk.scale.x = radius * 2.0f;
            mk.scale.y = radius * 2.0f;
            mk.scale.z = length;
          }
          break;
        }
        case shapes::MESH:
        {
          auto mesh_body = dynamic_cast<const robot_self_filter::bodies::ConvexMesh *>(body);
          if (mesh_body)
          {
            mk.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            mk.scale.x = mk.scale.y = mk.scale.z = 1.0f;

            const auto &verts = mesh_body->getScaledVertices();
            const auto &tris = mesh_body->getTriangles();
            mk.points.reserve(tris.size());
            for (size_t t_i = 0; t_i < tris.size(); ++t_i)
            {
              geometry_msgs::msg::Point p;
              p.x = verts[tris[t_i]].x();
              p.y = verts[tris[t_i]].y();
              p.z = verts[tris[t_i]].z();
              mk.points.push_back(p);
            }
          }
          break;
        }
        default:
          break;
        }
        marker_array.markers.push_back(mk);
      }

      marker_pub_->publish(marker_array);
      RCLCPP_DEBUG(this->get_logger(), "Published %zu collision shapes", marker_array.markers.size());
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<filters::SelfFilterInterface> self_filter_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<gs_ros_interfaces::msg::PointCloud2WithIndex>::SharedPtr sub_custom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublisher_;
    rclcpp::Publisher<gs_ros_interfaces::msg::PointCloud2WithIndex>::SharedPtr pointCloudPublisherCustom_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::string sensor_frame_;
    bool use_rgb_;
    SensorType sensor_type_;
    int max_queue_size_;
    std::vector<std::string> frames_;
    std::string in_topic_;
    bool use_custom_pointcloud_msg_;
    
    // Downsampling parameters
    bool enable_downsampling_;
    double voxel_leaf_size_;
    int max_points_;
    std::string downsample_method_;
    
    // Ground filtering parameters
    bool enable_ground_filtering_;
    double ground_z_threshold_;
    std::string target_frame_;
  };

} // namespace robot_self_filter

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_self_filter::SelfFilterNode>();
  node->initSelfFilter();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
