#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using FollowJointTrajectoryAction = control_msgs::action::FollowJointTrajectory;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("demo_motion");
  auto logger = node->get_logger();

  RCLCPP_INFO(logger, "Connecting to MoveGroup (waits up to 30 s in constructor)...");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "arm");

  RCLCPP_INFO(logger, "MoveGroup interface ready.");

  auto action_client = rclcpp_action::create_client<FollowJointTrajectoryAction>(
    node, "/arm_controller/follow_joint_trajectory");

  RCLCPP_INFO(logger, "Waiting for arm_controller action server...");
  if (!action_client->wait_for_action_server(std::chrono::seconds(15))) {
    RCLCPP_ERROR(logger, "arm_controller action server not available");
    return 1;
  }
  RCLCPP_INFO(logger, "arm_controller action server ready.");

  RCLCPP_INFO(logger, "Holding still for 30 s before motions begin...");
  std::this_thread::sleep_for(std::chrono::seconds(30));

  struct Pose {
    std::string name;
    std::vector<double> joints;
  };

  // Joint order follows the "arm" group: j1_joint, j2_joint, j3_joint,
  // ee_base_joint_x (wrist pitch, [0, pi]), ee_base_joint_y (wrist roll).
  std::vector<Pose> sequence = {
    {"upright",       {0.0, 0.2, 0.0, 0.3, 0.0}},
    {"reach_forward", {0.0, -0.8, -0.8, 0.6, 0.0}},
    {"swing_left",    {0.8, -0.4, -0.4, 0.4, 0.5}},
    {"swing_right",   {-0.8, -0.4, -0.4, 0.4, -0.5}},
    {"upright",       {0.0, 0.2, 0.0, 0.3, 0.0}},
  };

  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  for (const auto & pose : sequence) {
    RCLCPP_INFO(logger, "Planning for '%s' ...", pose.name.c_str());
    move_group.setJointValueTarget(pose.joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
      RCLCPP_ERROR(logger, "Planning failed for '%s'.", pose.name.c_str());
      return 1;
    }

    auto & traj = plan.trajectory_.joint_trajectory;
    auto & points = traj.points;
    if (points.size() >= 2) {
      if (points[1].time_from_start == rclcpp::Duration(0, 0)) {
        RCLCPP_WARN(logger, "Trajectory timing is flat; regenerating time_from_start.");
        double total_dur = 2.0;
        for (size_t i = 0; i < points.size(); ++i) {
          points[i].time_from_start = rclcpp::Duration::from_seconds(total_dur * i / (points.size() - 1));
        }
      }
    }

    RCLCPP_INFO(logger, "Executing '%s' via action server...", pose.name.c_str());

    auto goal = FollowJointTrajectoryAction::Goal();
    goal.trajectory = traj;

    auto send_goal_future = action_client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(
          node, send_goal_future, std::chrono::seconds(5))
        != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Failed to send goal for '%s'.", pose.name.c_str());
      return 1;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(logger, "Goal for '%s' was rejected.", pose.name.c_str());
      return 1;
    }

    auto result_future = action_client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(
          node, result_future, std::chrono::seconds(30))
        != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Execution result not received for '%s'.", pose.name.c_str());
      return 1;
    }

    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(logger, "Execution failed for '%s'.", pose.name.c_str());
      return 1;
    }

    RCLCPP_INFO(logger, "Reached '%s'.", pose.name.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_INFO(logger, "Demo motion complete.");
  rclcpp::shutdown();
  return 0;
}
