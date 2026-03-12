#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>

using namespace std::chrono_literals;

/**
 * PickPlaceServer
 *
 * Listens on /perception/grasp_pose (geometry_msgs/PoseStamped).
 * On each new pose, executes a pick sequence via MoveIt2 MoveGroupInterface:
 *   1. Move to pre-grasp pose (offset along approach axis)
 *   2. Move to grasp pose
 *   3. Close gripper
 *   4. Retreat
 *   5. Move to place pose
 *   6. Open gripper
 *
 * Parameters:
 *   arm_planning_group   (string)  : MoveIt planning group name, default "ur_arm"
 *   gripper_planning_group (string): MoveIt gripper group, default "gripper"
 *   approach_distance_m  (double)  : Pre-grasp standoff in metres, default 0.10
 *   place_x/y/z          (double)  : Fixed place location in base_link frame
 */
class PickPlaceServer : public rclcpp::Node
{
public:
  PickPlaceServer()
  : Node("pick_place_server",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    declare_parameter("arm_planning_group", "ur_manipulator");
    declare_parameter("gripper_planning_group", "gripper");
    declare_parameter("approach_distance_m", 0.10);
    declare_parameter("place_x", 0.4);
    declare_parameter("place_y", 0.3);
    declare_parameter("place_z", 0.3);

    arm_group_name_     = get_parameter("arm_planning_group").as_string();
    gripper_group_name_ = get_parameter("gripper_planning_group").as_string();
    approach_dist_      = get_parameter("approach_distance_m").as_double();
    place_x_            = get_parameter("place_x").as_double();
    place_y_            = get_parameter("place_y").as_double();
    place_z_            = get_parameter("place_z").as_double();

    // Subscribe to perception output — one shot per detection
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/perception/grasp_pose", 10,
      std::bind(&PickPlaceServer::graspPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "PickPlaceServer ready. Waiting for grasp pose...");
  }

private:
  void graspPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (executing_) {
      RCLCPP_WARN(get_logger(), "Already executing pick — ignoring new pose");
      return;
    }
    executing_ = true;
    RCLCPP_INFO(get_logger(), "Received grasp pose. Starting pick sequence.");

    // MoveGroupInterface must be constructed after node is spinning — use a thread
    std::thread([this, msg]() {
      executePick(*msg);
      executing_ = false;
    }).detach();
  }

  void executePick(const geometry_msgs::msg::PoseStamped & grasp_pose)
  {
    // ── Init MoveIt interfaces ─────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface arm(
      shared_from_this(), arm_group_name_);
    moveit::planning_interface::MoveGroupInterface gripper(
      shared_from_this(), gripper_group_name_);
    moveit::planning_interface::PlanningSceneInterface psi;

    arm.setPlanningTime(5.0);
    arm.setMaxVelocityScalingFactor(0.3);
    arm.setMaxAccelerationScalingFactor(0.2);

    // ── 1. Open gripper ───────────────────────────────────────────────────
    if (!setGripper(gripper, /*open=*/true)) return;

    // ── 2. Pre-grasp pose (approach from above) ───────────────────────────
    geometry_msgs::msg::PoseStamped pre_grasp = grasp_pose;
    pre_grasp.pose.position.z += approach_dist_;

    arm.setPoseTarget(pre_grasp);
    if (!planAndExecute(arm, "pre-grasp")) return;

    // ── 3. Grasp pose (straight down approach via Cartesian path) ─────────
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(grasp_pose.pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    if (fraction < 0.9) {
      RCLCPP_ERROR(get_logger(),
        "Cartesian approach only %.0f%% complete — aborting", fraction * 100.0);
      return;
    }
    arm.execute(trajectory);
    RCLCPP_INFO(get_logger(), "At grasp pose");

    // ── 4. Close gripper ──────────────────────────────────────────────────
    if (!setGripper(gripper, /*open=*/false)) return;
    rclcpp::sleep_for(500ms);  // let fingers settle

    // ── 5. Retreat (reverse Cartesian) ────────────────────────────────────
    geometry_msgs::msg::PoseStamped retreat = grasp_pose;
    retreat.pose.position.z += approach_dist_;
    std::vector<geometry_msgs::msg::Pose> retreat_wp = {retreat.pose};
    moveit_msgs::msg::RobotTrajectory retreat_traj;
    arm.computeCartesianPath(retreat_wp, 0.01, 0.0, retreat_traj);
    arm.execute(retreat_traj);

    // ── 6. Move to place location ─────────────────────────────────────────
    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header.frame_id = "base_link";
    place_pose.pose.position.x = place_x_;
    place_pose.pose.position.y = place_y_;
    place_pose.pose.position.z = place_z_;
    place_pose.pose.orientation.w = 1.0;  // upright

    arm.setPoseTarget(place_pose);
    if (!planAndExecute(arm, "place")) return;

    // ── 7. Open gripper (release) ─────────────────────────────────────────
    setGripper(gripper, /*open=*/true);
    rclcpp::sleep_for(300ms);

    // ── 8. Return to home ─────────────────────────────────────────────────
    arm.setNamedTarget("home");
    planAndExecute(arm, "home");

    RCLCPP_INFO(get_logger(), "Pick-and-place complete!");
  }

  bool planAndExecute(
    moveit::planning_interface::MoveGroupInterface & group,
    const std::string & stage)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = group.plan(plan);
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Planning failed at stage: %s", stage.c_str());
      return false;
    }
    group.execute(plan);
    RCLCPP_INFO(get_logger(), "Stage complete: %s", stage.c_str());
    return true;
  }

  bool setGripper(
    moveit::planning_interface::MoveGroupInterface & gripper,
    bool open)
  {
    gripper.setNamedTarget(open ? "open" : "closed");
    return planAndExecute(gripper, open ? "open gripper" : "close gripper");
  }

  // Parameters
  std::string arm_group_name_;
  std::string gripper_group_name_;
  double approach_dist_;
  double place_x_, place_y_, place_z_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  std::atomic<bool> executing_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
