#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>
#include <string>

namespace perception
{

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,   // aligned depth
    sensor_msgs::msg::Image,   // segmentation mask
    sensor_msgs::msg::CameraInfo
>;

class PointCloudProcessor : public rclcpp::Node
{
public:
  explicit PointCloudProcessor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Synchronized callback: depth + mask + camera_info arrive together
  void syncedCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & mask_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  // Build a masked point cloud from depth image + binary mask
  pcl::PointCloud<pcl::PointXYZ>::Ptr buildMaskedCloud(
    const cv::Mat & depth_image,
    const cv::Mat & binary_mask,
    float fx, float fy, float cx, float cy,
    float depth_scale,
    float min_depth_m, float max_depth_m);

  // Estimate grasp pose: centroid + PCA-based orientation
  std::optional<geometry_msgs::msg::PoseStamped> estimateGraspPose(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::string & frame_id,
    const rclcpp::Time & stamp);

  // Parameters
  int target_instance_id_;   // which mask instance to pick (1-indexed)
  float depth_scale_;        // RealSense depth unit → meters (typically 0.001)
  float min_depth_m_;
  float max_depth_m_;
  std::string target_frame_; // TF frame to express pose in (e.g. "base_link")

  // Subscribers (synchronized)
  message_filters::Subscriber<sensor_msgs::msg::Image>      depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image>      mask_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   cloud_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace perception
