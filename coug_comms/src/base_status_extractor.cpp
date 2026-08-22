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

#include <rclcpp_components/register_node_macro.hpp>

namespace coug_comms {

BaseStatusExtractorNode::BaseStatusExtractorNode(const rclcpp::NodeOptions& options)
    : Node("base_status_extractor_node", options) {
  param_listener_ =
      std::make_shared<base_status_extractor_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  this->declare_parameter<std::vector<std::string>>("agent_namespaces", std::vector<std::string>{});
  const auto agent_namespaces = this->get_parameter("agent_namespaces").as_string_array();

  for (const auto& aname : agent_namespaces) {
    registerAgent(aname);
  }

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void BaseStatusExtractorNode::statusCallback(
    const std::string& aname, const coug_interfaces::msg::AgentStatus::SharedPtr msg) {
  auto it = agents_.find(aname);
  if (it == agents_.end()) {
    return;
  }

  it->second.odom_pub->publish(convertToOdom(aname, msg));
  it->second.depth_pub->publish(convertToDepth(msg));
  it->second.imu_pub->publish(convertToImu(msg));
}

void BaseStatusExtractorNode::registerAgent(const std::string& aname) {
  AgentEntry a;
  a.name = aname;

  a.odom_pub = create_publisher<nav_msgs::msg::Odometry>("/" + aname + "/" + params_.odom_topic,
                                                         rclcpp::SystemDefaultsQoS());

  a.depth_pub = create_publisher<nav_msgs::msg::Odometry>("/" + aname + "/" + params_.depth_topic,
                                                          rclcpp::SystemDefaultsQoS());

  a.imu_pub = create_publisher<sensor_msgs::msg::Imu>("/" + aname + "/" + params_.imu_topic,
                                                      rclcpp::SystemDefaultsQoS());

  a.status_sub = create_subscription<coug_interfaces::msg::AgentStatus>(
      "/" + aname + "/" + params_.status_topic, rclcpp::SystemDefaultsQoS(),
      [this, aname](const coug_interfaces::msg::AgentStatus::SharedPtr msg) {
        statusCallback(aname, msg);
      });

  agents_.emplace(aname, std::move(a));
  RCLCPP_INFO(get_logger(), "Registered extractor for agent '%s'.", aname.c_str());
}

nav_msgs::msg::Odometry BaseStatusExtractorNode::convertToOdom(
    const std::string& aname, const coug_interfaces::msg::AgentStatus::SharedPtr msg) {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header = msg->header;
  odom_msg.header.frame_id = "map";
  odom_msg.child_frame_id = aname + "/base_link";
  odom_msg.pose.pose = msg->local_odometry;
  odom_msg.pose.covariance = msg->odometry_covariance;
  return odom_msg;
}

nav_msgs::msg::Odometry BaseStatusExtractorNode::convertToDepth(
    const coug_interfaces::msg::AgentStatus::SharedPtr msg) {
  nav_msgs::msg::Odometry depth_msg;
  depth_msg.header = msg->header;
  depth_msg.pose.pose.position.z = msg->pressure_depth;
  return depth_msg;
}

sensor_msgs::msg::Imu BaseStatusExtractorNode::convertToImu(
    const coug_interfaces::msg::AgentStatus::SharedPtr msg) {
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header = msg->header;
  imu_msg.orientation = msg->imu_orientation;
  return imu_msg;
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::BaseStatusExtractorNode)
