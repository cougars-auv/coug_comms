// Copyright 2026 BYU FROST Lab
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

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "coug_comms/agent_status_bundler_parameters.hpp"
#include "coug_interfaces/msg/agent_status.hpp"

namespace coug_comms {

class AgentStatusBundlerNode : public rclcpp::Node {
 public:
  explicit AgentStatusBundlerNode(const rclcpp::NodeOptions& options);

 private:
  // --- Callbacks ---
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  void depthCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

  // --- Helpers ---
  void publishStatus();

  // --- ROS Interfaces ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<coug_interfaces::msg::AgentStatus>::SharedPtr status_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Parameters ---
  std::shared_ptr<agent_status_bundler_node::ParamListener> param_listener_;
  agent_status_bundler_node::Params params_;

  // --- State ---
  nav_msgs::msg::Odometry::ConstSharedPtr last_odom_;
  nav_msgs::msg::Odometry::ConstSharedPtr last_depth_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
};

}  // namespace coug_comms
