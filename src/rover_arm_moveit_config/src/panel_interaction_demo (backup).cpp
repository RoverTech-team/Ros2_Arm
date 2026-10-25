#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

using FollowJointTrajectoryAction = control_msgs::action::FollowJointTrajectory;
using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;

struct PanelTarget {
  std::string name;
  double x, y, z;
  // Fallback joint-space target in the "arm" group order:
  // j1_joint, j2_joint, j3_joint, ee_base_joint_x, ee_base_joint_y.
  double j1, j2, j3, jx, jy;
};

// World-frame positions of panel elements, plus fallback joint-space targets.
// World positions are computed from panel_base frame via world_to_panel_base:
//   x = 1.5 - panel_y,  y = panel_x,  z = panel_z
// The fallback joints are approximate; the primary approach uses a Cartesian
// target on tool0. jx (ee_base_joint_x) is held at 0.0; jy (ee_base_joint_y)
// reuses the old wrist-roll angle.
static const std::vector<PanelTarget> kTargets = {
  {"electromagnet",   1.470, 0.240, 0.050,  -1.099, -0.806, 1.229, 0.0, 0.0},
  {"socket_grey",     1.490, 0.130, 0.135,  -1.312, -0.959, 1.324, 0.0, 0.0},
  {"socket_pink_2",   1.465, 0.265, 0.135,  -1.053, -0.500, 1.000, 0.0, 0.0},
  {"main_switch",     1.480, 0.050, 0.220,  -1.467, -0.549, 1.352, 0.0, 0.0},
  {"lever_switch_2",  1.480, 0.165, 0.220,  -1.240, -0.256, 0.907, 0.0, 0.0},
  {"lever_switch_4",  1.480, 0.280, 0.220,  -1.043, -0.387, 1.045, 0.0, 0.0},
  {"power_switch_1",  1.475, 0.210, 0.300,  -1.155, -0.115, 0.728, 0.0, 0.0},
  {"indicator_3",     1.495, 0.165, 0.375,  -1.249, -0.173, 0.798, 0.0, 0.0},
  {"aruco_tag_1",     1.499, 0.280, 0.405,  -1.060, -0.400, 1.000, 0.0, 0.0},
};

static bool driveToPose(
    const std::string & frame_id,
    double x, double y, double yaw,
    rclcpp_action::Client<NavigateToPoseAction>::SharedPtr & client,
    rclcpp::Executor & executor,
    rclcpp::Logger & logger)
{
  if (!client->action_server_is_ready()) {
    RCLCPP_WARN(logger, "Nav2 action server not ready, skipping drive.");
    return false;
  }
  auto goal = NavigateToPoseAction::Goal();
  goal.pose.header.frame_id = frame_id;
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.pose.pose.orientation = tf2::toMsg(q);
  RCLCPP_INFO(logger, "Driving base to (%.2f, %.2f, %.2f) in frame '%s'...",
              x, y, yaw, frame_id.c_str());
  auto send_future = client->async_send_goal(goal);
  if (executor.spin_until_future_complete(send_future, std::chrono::seconds(5))
      != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(logger, "Failed to send Nav2 goal.");
    return false;
  }
  auto goal_handle = send_future.get();
  if (!goal_handle) {
    RCLCPP_WARN(logger, "Nav2 goal was rejected.");
    return false;
  }
  auto result_future = client->async_get_result(goal_handle);
  if (executor.spin_until_future_complete(result_future, std::chrono::seconds(60))
      != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(logger, "Nav2 navigation timed out.");
    return false;
  }
  auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(logger, "Nav2 navigation failed.");
    return false;
  }
  RCLCPP_INFO(logger, "Base repositioning complete.");
  return true;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("panel_interaction_demo");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto logger = node->get_logger();

  node->declare_parameter<bool>("arm_only", false);
  node->declare_parameter<bool>("force_panel_facing", true);
  node->declare_parameter<std::string>("trajectory_action_topic", "/arm/arm_controller/follow_joint_trajectory");
  bool arm_only = node->get_parameter("arm_only").as_bool();
  bool force_panel_facing = node->get_parameter("force_panel_facing").as_bool();
  std::string traj_topic = node->get_parameter("trajectory_action_topic").as_string();

  RCLCPP_INFO(logger, "Configuration: arm_only=%s, force_panel_facing=%s, trajectory_action_topic=%s",
              arm_only ? "true" : "false", force_panel_facing ? "true" : "false", traj_topic.c_str());

  RCLCPP_INFO(logger, "Creating action client for %s...", traj_topic.c_str());
  auto action_client = rclcpp_action::create_client<FollowJointTrajectoryAction>(
    node, traj_topic);

  RCLCPP_INFO(logger, "Waiting for arm_controller action server (with executor spinning)...");
  {
    auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < timeout) {
      if (action_client->action_server_is_ready()) {
        RCLCPP_INFO(logger, "action_server_is_ready() returned true for %s", traj_topic.c_str());
        break;
      }
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!action_client->action_server_is_ready()) {
      RCLCPP_WARN(logger, "Action server not ready on %s, trying fallback /arm_controller/follow_joint_trajectory...", traj_topic.c_str());
      traj_topic = "/arm_controller/follow_joint_trajectory";
      action_client = rclcpp_action::create_client<FollowJointTrajectoryAction>(node, traj_topic);
      auto fallback_timeout = std::chrono::steady_clock::now() + std::chrono::seconds(15);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < fallback_timeout) {
        if (action_client->action_server_is_ready()) {
          RCLCPP_INFO(logger, "action_server_is_ready() returned true for fallback %s", traj_topic.c_str());
          break;
        }
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
    if (!action_client->action_server_is_ready()) {
      RCLCPP_ERROR(logger, "arm_controller action server not available on either topic after timeout");
    }
  }
  RCLCPP_INFO(logger, "arm_controller action server check complete.");

  // ── Nav2 NavigateToPose action client (optional if arm_only) ──
  rclcpp_action::Client<NavigateToPoseAction>::SharedPtr nav_client = nullptr;
  if (!arm_only) {
    RCLCPP_INFO(logger, "Creating action client for /navigate_to_pose...");
    nav_client = rclcpp_action::create_client<NavigateToPoseAction>(
      node, "/navigate_to_pose");
    RCLCPP_INFO(logger, "Waiting for Nav2 navigate_to_pose action server...");
    {
      auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(20);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < timeout) {
        if (nav_client->action_server_is_ready()) break;
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (!nav_client->action_server_is_ready()) {
        RCLCPP_WARN(logger, "Nav2 action server not available after 20s. Running in arm-only mode.");
        arm_only = true;
      } else {
        RCLCPP_INFO(logger, "Nav2 action server ready.");
      }
    }
  }

  RCLCPP_INFO(logger, "Connecting to MoveGroup...");
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "arm");
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);
  move_group.setPlanningTime(5.0);
  RCLCPP_INFO(logger, "MoveGroup interface ready.");

  // Facing orientation: tool0 pointing towards +x (normal to the maintenance panel at x=1.5)
  // Identity orientation in world frame has tool0 pointing along +x with upright alignment.
  geometry_msgs::msg::Quaternion facing_orientation;
  facing_orientation.x = 0.0;
  facing_orientation.y = 0.0;
  facing_orientation.z = 0.0;
  facing_orientation.w = 1.0;

  if (force_panel_facing) {
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "tool0";
    ocm.header.frame_id = "world";
    ocm.orientation = facing_orientation;
    // Enforce orientation facing the panel with allowable deviation in pitch/roll/yaw
    ocm.absolute_x_axis_tolerance = 0.6; // ~34 deg tolerance roll around approach axis
    ocm.absolute_y_axis_tolerance = 0.4; // pitch tolerance
    ocm.absolute_z_axis_tolerance = 0.4; // yaw tolerance
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.name = "face_maintenance_panel";
    path_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(path_constraints);
    RCLCPP_INFO(logger, "Path constraints applied: manipulator forced to face maintenance panel.");
  }

  // Start from a known safe pose (upright)
  {
    RCLCPP_INFO(logger, "Moving to upright start pose...");
    move_group.setJointValueTarget({0.0, 0.2, 0.0, 0.0, 0.0});
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      auto & traj = plan.trajectory.joint_trajectory;
      auto & points = traj.points;
      if (points.size() >= 2 && points[1].time_from_start == rclcpp::Duration(0, 0)) {
        for (size_t i = 0; i < points.size(); ++i) {
          points[i].time_from_start = rclcpp::Duration::from_seconds(2.0 * i / (points.size() - 1));
        }
      }
      auto goal = FollowJointTrajectoryAction::Goal();
      goal.trajectory = traj;
      auto goal_future = action_client->async_send_goal(goal);
      if (executor.spin_until_future_complete(goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(logger, "Failed to send initial goal.");
      } else {
        auto goal_handle = goal_future.get();
        if (goal_handle) {
          auto result_future = action_client->async_get_result(goal_handle);
          executor.spin_until_future_complete(result_future);
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // ── Base navigation to working position near panel (if not arm-only) ──
  if (!arm_only && nav_client && nav_client->action_server_is_ready()) {
    RCLCPP_INFO(logger, "--- Base navigation phase ---");
    driveToPose("map", 1.2, 0.0, 0.0, nav_client, executor, logger);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  int succeeded = 0;
  int failed = 0;

  for (const auto & tgt : kTargets) {
    RCLCPP_INFO(logger, "--- Approaching '%s' (%.3f, %.3f, %.3f) ---",
                tgt.name.c_str(), tgt.x, tgt.y, tgt.z);

    if (force_panel_facing) {
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = tgt.x;
      target_pose.position.y = tgt.y;
      target_pose.position.z = tgt.z;
      target_pose.orientation = facing_orientation;
      move_group.setPoseReferenceFrame("odom");

      move_group.setPoseTarget(target_pose, "tool0");
    } else {
      move_group.setPositionTarget(tgt.x, tgt.y, tgt.z, "tool0");
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group.plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger, "Pose planning failed, trying position target...");
      move_group.setPositionTarget(tgt.x, tgt.y, tgt.z, "tool0");
      plan_result = move_group.plan(plan);
    }

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger, "Cartesian planning failed, trying joint-space fallback...");
      move_group.setJointValueTarget({tgt.j1, tgt.j2, tgt.j3, tgt.jx, tgt.jy});
      plan_result = move_group.plan(plan);
    }

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger, "Planning failed for '%s', skipping.", tgt.name.c_str());
      ++failed;
      continue;
    }

    auto & traj = plan.trajectory.joint_trajectory;
    auto & points = traj.points;
    if (points.size() >= 2) {
      if (points[1].time_from_start == rclcpp::Duration(0, 0)) {
        double total_dur = 2.0;
        for (size_t i = 0; i < points.size(); ++i) {
          points[i].time_from_start = rclcpp::Duration::from_seconds(
              total_dur * i / (points.size() - 1));
        }
      }
    }

    auto goal = FollowJointTrajectoryAction::Goal();
    goal.trajectory = traj;

    auto send_goal_future = action_client->async_send_goal(goal);
    if (executor.spin_until_future_complete(send_goal_future, std::chrono::seconds(5))
        != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(logger, "Failed to send goal for '%s'.", tgt.name.c_str());
      ++failed;
      continue;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(logger, "Goal for '%s' was rejected.", tgt.name.c_str());
      ++failed;
      continue;
    }

    auto result_future = action_client->async_get_result(goal_handle);
    if (executor.spin_until_future_complete(result_future, std::chrono::seconds(30))
        != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(logger, "Execution result not received for '%s'.", tgt.name.c_str());
      ++failed;
      continue;
    }

    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(logger, "Execution failed for '%s'.", tgt.name.c_str());
      ++failed;
      continue;
    }

    RCLCPP_INFO(logger, "Reached '%s'.", tgt.name.c_str());
    ++succeeded;
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
  }

  RCLCPP_INFO(logger, "Demo complete: %d succeeded, %d failed.", succeeded, failed);

  if (failed > 0) {
    RCLCPP_WARN(logger, "Some targets were not reached.");
  }

  rclcpp::shutdown();
  return failed > 0 ? 1 : 0;
}
