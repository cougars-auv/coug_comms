// Copyright (c) 2026 BYU FROST Lab
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

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "coug_comms/base_status_extractor_parameters.hpp"
#include "coug_interfaces/msg/agent_status.hpp"

namespace coug_comms {

class BaseStatusExtractorNode : public rclcpp::Node {
 public:
  explicit BaseStatusExtractorNode(const rclcpp::NodeOptions& options);

 private:
  struct AgentEntry {
    rclcpp::Subscription<coug_interfaces::msg::AgentStatus>::SharedPtr status_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr depth_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  };

  // --- Callbacks ---
  void statusCallback(const std::string& agent_name,
                      const coug_interfaces::msg::AgentStatus::SharedPtr& msg);

  // --- Helpers ---
  void registerAgent(const std::string& agent_name);

  static nav_msgs::msg::Odometry convertToOdom(
      const std::string& agent_name, const coug_interfaces::msg::AgentStatus::SharedPtr& msg);

  static nav_msgs::msg::Odometry convertToDepth(
      const coug_interfaces::msg::AgentStatus::SharedPtr& msg);

  static sensor_msgs::msg::Imu convertToImu(
      const coug_interfaces::msg::AgentStatus::SharedPtr& msg);

  // --- Parameters ---
  std::shared_ptr<base_status_extractor_node::ParamListener> param_listener_;
  base_status_extractor_node::Params params_;

  // --- State ---
  std::unordered_map<std::string, AgentEntry> agents_;
};

}  // namespace coug_comms
