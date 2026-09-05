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

#include "coug_comms/base_status_extractor.hpp"

#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <utility>

#include "coug_comms/base_status_extractor_parameters.hpp"
#include "coug_interfaces/msg/agent_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace coug_comms {

using coug_interfaces::msg::AgentStatus;

namespace {

std::string build_name(const std::string& agent, const std::string& sub) {
  return "/" + agent + "/" + sub;
}

}  // namespace

BaseStatusExtractorNode::BaseStatusExtractorNode(const rclcpp::NodeOptions& options)
    : Node("base_status_extractor_node", options) {
  param_listener_ =
      std::make_shared<base_status_extractor_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  for (const auto& agent_name : params_.agent_list) {
    registerAgent(agent_name);
  }

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void BaseStatusExtractorNode::statusCallback(const std::string& agent_name,
                                             const AgentStatus::ConstSharedPtr& msg) {
  auto it = agents_.find(agent_name);
  if (it == agents_.end()) {
    return;
  }

  auto& agent = it->second;
  agent.odom_pub->publish(convertToOdom(agent_name, msg));
  agent.depth_pub->publish(convertToDepth(msg));
  agent.imu_pub->publish(convertToImu(msg));
}

void BaseStatusExtractorNode::registerAgent(const std::string& agent_name) {
  AgentEntry agent;
  agent.odom_pub = create_publisher<nav_msgs::msg::Odometry>(
      build_name(agent_name, params_.odom_topic), rclcpp::SystemDefaultsQoS());

  agent.depth_pub = create_publisher<nav_msgs::msg::Odometry>(
      build_name(agent_name, params_.depth_topic), rclcpp::SystemDefaultsQoS());

  agent.imu_pub = create_publisher<sensor_msgs::msg::Imu>(build_name(agent_name, params_.imu_topic),
                                                          rclcpp::SystemDefaultsQoS());

  agent.status_sub = create_subscription<AgentStatus>(
      build_name(agent_name, params_.status_topic), rclcpp::SystemDefaultsQoS(),
      [this, agent_name](const AgentStatus::ConstSharedPtr& msg) {
        statusCallback(agent_name, msg);
      });

  agents_.emplace(agent_name, std::move(agent));
  RCLCPP_INFO(get_logger(), "Registered agent '%s'.", agent_name.c_str());
}

nav_msgs::msg::Odometry BaseStatusExtractorNode::convertToOdom(
    const std::string& agent_name, const AgentStatus::ConstSharedPtr& msg) {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header = msg->header;
  odom_msg.header.frame_id = "map";
  odom_msg.child_frame_id = agent_name + "/base_link";
  odom_msg.pose.pose = msg->local_odometry;
  odom_msg.pose.covariance = msg->odometry_covariance;
  return odom_msg;
}

nav_msgs::msg::Odometry BaseStatusExtractorNode::convertToDepth(
    const AgentStatus::ConstSharedPtr& msg) {
  nav_msgs::msg::Odometry depth_msg;
  depth_msg.header = msg->header;
  depth_msg.pose.pose.position.z = msg->pressure_depth;
  return depth_msg;
}

sensor_msgs::msg::Imu BaseStatusExtractorNode::convertToImu(
    const AgentStatus::ConstSharedPtr& msg) {
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header = msg->header;
  imu_msg.orientation = msg->imu_orientation;
  return imu_msg;
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::BaseStatusExtractorNode)
