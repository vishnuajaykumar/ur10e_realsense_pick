#include "perception/pointcloud_processor.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <cmath>

namespace perception
{

PointCloudProcessor::PointCloudProcessor(const rclcpp::NodeOptions & options)
: Node("pointcloud_processor", options)
{
  // Declare parameters with defaults
  declare_parameter("target_instance_id", 1);
  declare_parameter("depth_scale", 0.001f);       // RealSense default: 1mm per unit
  declare_parameter("min_depth_m", 0.1f);
  declare_parameter("max_depth_m", 2.0f);
  declare_parameter("target_frame", "base_link");

  get_parameter("target_instance_id", target_instance_id_);
  get_parameter("depth_scale", depth_scale_);
  get_parameter("min_depth_m", min_depth_m_);
  get_parameter("max_depth_m", max_depth_m_);
  get_parameter("target_frame", target_frame_);

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Synchronized subscribers (approximate time, 50ms tolerance)
  depth_sub_.subscribe(this, "/camera/aligned_depth_to_color/image_raw");
  mask_sub_.subscribe(this, "/segmentation/masks");
  info_sub_.subscribe(this, "/camera/color/camera_info");

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), depth_sub_, mask_sub_, info_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
  sync_->registerCallback(
    std::bind(&PointCloudProcessor::syncedCallback, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Publishers
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/perception/grasp_pose", 10);
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/perception/object_cloud", 10);

  RCLCPP_INFO(get_logger(),
    "PointCloudProcessor ready. Targeting instance id=%d in frame=%s",
    target_instance_id_, target_frame_.c_str());
}

void PointCloudProcessor::syncedCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & mask_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  // ── 1. Decode images ──────────────────────────────────────────────────────
  cv_bridge::CvImagePtr depth_ptr, mask_ptr;
  try {
    depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    mask_ptr  = cv_bridge::toCvCopy(mask_msg,  sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // ── 2. Extract binary mask for the target instance ────────────────────────
  cv::Mat binary_mask = (mask_ptr->image == static_cast<uint8_t>(target_instance_id_));
  if (cv::countNonZero(binary_mask) == 0) {
    RCLCPP_DEBUG(get_logger(), "Instance id=%d not found in mask", target_instance_id_);
    return;
  }

  // ── 3. Camera intrinsics from CameraInfo ──────────────────────────────────
  const float fx = static_cast<float>(info_msg->k[0]);
  const float fy = static_cast<float>(info_msg->k[4]);
  const float cx = static_cast<float>(info_msg->k[2]);
  const float cy = static_cast<float>(info_msg->k[5]);

  // ── 4. Build masked point cloud ───────────────────────────────────────────
  auto cloud = buildMaskedCloud(
    depth_ptr->image, binary_mask,
    fx, fy, cx, cy,
    depth_scale_, min_depth_m_, max_depth_m_);

  if (cloud->empty()) {
    RCLCPP_WARN(get_logger(), "Empty point cloud after masking — skipping");
    return;
  }

  // Publish raw masked cloud for visualization
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header = depth_msg->header;
  cloud_pub_->publish(cloud_msg);

  // ── 5. Estimate grasp pose ────────────────────────────────────────────────
  auto pose_opt = estimateGraspPose(cloud, depth_msg->header.frame_id,
                                    rclcpp::Time(depth_msg->header.stamp));
  if (pose_opt.has_value()) {
    pose_pub_->publish(pose_opt.value());
    RCLCPP_INFO(get_logger(), "Published grasp pose for instance %d", target_instance_id_);
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::buildMaskedCloud(
  const cv::Mat & depth_image,
  const cv::Mat & binary_mask,
  float fx, float fy, float cx, float cy,
  float depth_scale,
  float min_depth_m, float max_depth_m)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->reserve(static_cast<size_t>(cv::countNonZero(binary_mask)));

  for (int v = 0; v < depth_image.rows; ++v) {
    for (int u = 0; u < depth_image.cols; ++u) {
      if (binary_mask.at<uint8_t>(v, u) == 0) continue;

      const uint16_t raw_depth = depth_image.at<uint16_t>(v, u);
      if (raw_depth == 0) continue;  // invalid depth

      const float z = raw_depth * depth_scale;
      if (z < min_depth_m || z > max_depth_m) continue;

      // Back-project pixel (u, v, z) → 3D point in camera frame
      pcl::PointXYZ pt;
      pt.x = (u - cx) * z / fx;
      pt.y = (v - cy) * z / fy;
      pt.z = z;
      cloud->push_back(pt);
    }
  }

  if (cloud->empty()) return cloud;

  // Voxel downsample to reduce noise and computation
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.005f, 0.005f, 0.005f);  // 5mm voxels
  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  vg.filter(*filtered);

  // Statistical outlier removal
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(filtered);
  sor.setMeanK(20);
  sor.setStddevMulThresh(1.5);
  auto cleaned = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  sor.filter(*cleaned);

  return cleaned;
}

std::optional<geometry_msgs::msg::PoseStamped> PointCloudProcessor::estimateGraspPose(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  if (cloud->size() < 10) {
    RCLCPP_WARN(get_logger(), "Too few points (%zu) for PCA", cloud->size());
    return std::nullopt;
  }

  // ── Centroid ──────────────────────────────────────────────────────────────
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);

  // ── PCA for orientation ───────────────────────────────────────────────────
  // Principal axis 0 → longest axis of object → approach direction
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloud);
  const Eigen::Matrix3f & eigenvectors = pca.getEigenVectors();

  // Build rotation matrix: x = principal axis, z = points toward camera (−z of camera)
  Eigen::Vector3f x_axis = eigenvectors.col(0).normalized();
  Eigen::Vector3f z_axis = Eigen::Vector3f(0, 0, -1);  // approach from above
  Eigen::Vector3f y_axis = z_axis.cross(x_axis).normalized();
  z_axis = x_axis.cross(y_axis).normalized();

  Eigen::Matrix3f rot;
  rot.col(0) = x_axis;
  rot.col(1) = y_axis;
  rot.col(2) = z_axis;

  Eigen::Quaternionf quat(rot);
  quat.normalize();

  // ── Build PoseStamped in camera frame ─────────────────────────────────────
  geometry_msgs::msg::PoseStamped pose_camera;
  pose_camera.header.stamp = stamp;
  pose_camera.header.frame_id = frame_id;
  pose_camera.pose.position.x = centroid[0];
  pose_camera.pose.position.y = centroid[1];
  pose_camera.pose.position.z = centroid[2];
  pose_camera.pose.orientation.x = quat.x();
  pose_camera.pose.orientation.y = quat.y();
  pose_camera.pose.orientation.z = quat.z();
  pose_camera.pose.orientation.w = quat.w();

  // ── Transform to target_frame (e.g. base_link) ───────────────────────────
  if (frame_id == target_frame_) {
    return pose_camera;
  }

  try {
    auto transformed = tf_buffer_->transform(pose_camera, target_frame_,
                                              tf2::durationFromSec(0.1));
    return transformed;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
    return std::nullopt;
  }
}

}  // namespace perception

// ── main ─────────────────────────────────────────────────────────────────────
#include <rclcpp/executors.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::PointCloudProcessor>());
  rclcpp::shutdown();
  return 0;
}
