// Copyright 2023 ros2_control Development Team
//
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

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("send_trajectory");
  auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/arm_controller/joint_trajectory", 10);

  // get robot description
  auto robot_param = rclcpp::Parameter();
  node->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
  node->get_parameter("robot_description", robot_param);
  auto robot_description = robot_param.as_string();

  // create kinematic chain
  KDL::Tree robot_tree;
  KDL::Chain chain;
  if (!kdl_parser::treeFromString(robot_description, robot_tree))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse robot_description into KDL tree.");
    return 1;
  }
  // Try the common ROS tip first; fallback to this arm's tip link.
  if (!robot_tree.getChain("arm_base_link", "tool0", chain) &&
    !robot_tree.getChain("arm_base_link", "j3", chain))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to build KDL chain from 'arm_base_link' to either 'tool0' or 'j3'.");
    return 1;
  }

  auto joint_positions = KDL::JntArray(chain.getNrOfJoints());
  auto joint_velocities = KDL::JntArray(chain.getNrOfJoints());
  auto twist = KDL::Twist();
  // create KDL solvers
  auto ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain, 0.0000001);

  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  // Use zero start stamp so the controller starts execution from reception time.
  // This avoids "non-zero start time ... ends in the past" when publisher and
  // controller run on different machines with unsynchronized clocks.
  trajectory_msg.header.stamp = builtin_interfaces::msg::Time();
  std::vector<std::string> chain_joint_names;
  for (unsigned int i = 0; i < chain.getNrOfSegments(); i++)
  {
    auto joint = chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::Fixed)
    {
      chain_joint_names.push_back(joint.getName());
    }
  }

  // New 5-DOF arm group: j1_joint, j2_joint, j3_joint, ee_base_joint_x, ee_base_joint_y.
  const std::vector<std::string> controlled_joint_names = {
    "j1_joint", "j2_joint", "j3_joint", "ee_base_joint_x", "ee_base_joint_y"};
  trajectory_msg.joint_names = controlled_joint_names;

  std::vector<size_t> controlled_joint_indices;
  controlled_joint_indices.reserve(controlled_joint_names.size());
  for (const auto & controlled_joint_name : controlled_joint_names)
  {
    auto it = std::find(chain_joint_names.begin(), chain_joint_names.end(), controlled_joint_name);
    if (it == chain_joint_names.end())
    {
      RCLCPP_ERROR(
        node->get_logger(), "Controlled joint '%s' not found in KDL chain.",
        controlled_joint_name.c_str());
      return 1;
    }
    controlled_joint_indices.push_back(std::distance(chain_joint_names.begin(), it));
  }

  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
  trajectory_point_msg.positions.resize(controlled_joint_names.size());
  trajectory_point_msg.velocities.resize(controlled_joint_names.size());

  double total_time = 3.0;
  int trajectory_len = 200;
  double dt = total_time / static_cast<double>(trajectory_len - 1);

  for (int i = 0; i < trajectory_len; i++)
  {
    // set endpoint twist
    double t = i / (static_cast<double>(trajectory_len - 1));
    twist.vel = KDL::Vector::Zero();
    twist.rot = KDL::Vector::Zero();
    // Mixed linear + angular command to excite all 5 arm joints for this geometry.
    twist.vel.x(0.20 * cos(2 * M_PI * t));
    twist.vel.z(0.12 * sin(2 * M_PI * t));
    twist.rot.x(0.35 * cos(2 * M_PI * t));

    // convert cart to joint velocities
    ik_vel_solver_->CartToJnt(joint_positions, twist, joint_velocities);

    // copy to trajectory_point_msg
    for (size_t joint_idx = 0; joint_idx < controlled_joint_indices.size(); joint_idx++)
    {
      const auto chain_joint_idx = controlled_joint_indices[joint_idx];
      trajectory_point_msg.positions[joint_idx] = joint_positions(chain_joint_idx);
      trajectory_point_msg.velocities[joint_idx] = joint_velocities(chain_joint_idx);
    }

    // integrate joint velocities
    joint_positions.data += joint_velocities.data * dt;

    // set timing information
    double time_point = total_time * t;
    double time_point_sec = std::floor(time_point);
    trajectory_point_msg.time_from_start.sec = static_cast<int>(time_point_sec);
    trajectory_point_msg.time_from_start.nanosec =
      static_cast<uint32_t>((time_point - time_point_sec) * 1E9);
    trajectory_msg.points.push_back(trajectory_point_msg);
  }

  // send zero velocities in the end
  auto & last_point_msg = trajectory_msg.points.back();
  std::fill(last_point_msg.velocities.begin(), last_point_msg.velocities.end(), 0.0);

  // Log per-joint command ranges to verify all controlled joints are being driven.
  std::vector<double> min_positions(
    controlled_joint_names.size(), std::numeric_limits<double>::max());
  std::vector<double> max_positions(
    controlled_joint_names.size(), std::numeric_limits<double>::lowest());
  std::vector<double> min_velocities(
    controlled_joint_names.size(), std::numeric_limits<double>::max());
  std::vector<double> max_velocities(
    controlled_joint_names.size(), std::numeric_limits<double>::lowest());

  for (const auto & point : trajectory_msg.points)
  {
    for (size_t i = 0; i < controlled_joint_names.size(); i++)
    {
      min_positions[i] = std::min(min_positions[i], point.positions[i]);
      max_positions[i] = std::max(max_positions[i], point.positions[i]);
      min_velocities[i] = std::min(min_velocities[i], point.velocities[i]);
      max_velocities[i] = std::max(max_velocities[i], point.velocities[i]);
    }
  }

  for (size_t i = 0; i < controlled_joint_names.size(); i++)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "%s command range: pos[%.6f, %.6f] vel[%.6f, %.6f]",
      controlled_joint_names[i].c_str(), min_positions[i], max_positions[i], min_velocities[i],
      max_velocities[i]);
  }

  auto started = node->now();
  while (pub->get_subscription_count() == 0)
  {
    if (node->now() - started > rclcpp::Duration(10, 0))
    {
      RCLCPP_ERROR(
        node->get_logger(), "No subscribers connected after waiting for 10 seconds. Exiting.");
      return 1;
    }
    RCLCPP_INFO(
      node->get_logger(), "Waiting for subscribers to connect to topic %s", pub->get_topic_name());
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_INFO(
    node->get_logger(), "Publishing trajectory with length %ld", trajectory_msg.points.size());

  pub->publish(trajectory_msg);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}