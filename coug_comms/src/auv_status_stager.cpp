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

/**
 * @file auv_status_stager.cpp
 * @brief Implementation of the AuvStatusStagerNode.
 * @author Nelson Durrant
 * @date June 2026
 */

#include "coug_comms/auv_status_stager.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include "coug_comms/utils/seatrac_enums.hpp"
#include "coug_comms/utils/status_codec.hpp"

namespace coug_comms {

using utils::BEACON_ALL;
using utils::CID_DAT_QUEUE_SET;

AuvStatusStagerNode::AuvStatusStagerNode(const rclcpp::NodeOptions& options)
    : Node("auv_status_stager_node", options) {
  param_listener_ =
      std::make_shared<auv_status_stager_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- ROS Interfaces ---
  modem_send_pub_ = create_publisher<seatrac_interfaces::msg::ModemSend>(
      params_.modem_send_topic, rclcpp::SystemDefaultsQoS());
  status_sub_ = create_subscription<coug_interfaces::msg::AgentStatus>(
      params_.status_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&AuvStatusStagerNode::statusCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void AuvStatusStagerNode::statusCallback(const coug_interfaces::msg::AgentStatus::SharedPtr msg) {
  seatrac_interfaces::msg::ModemSend send;
  send.msg_id = CID_DAT_QUEUE_SET;
  send.dest_id = BEACON_ALL;
  send.packet_len = utils::encodeStatus(*msg, send.packet_data.data());
  modem_send_pub_->publish(send);
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::AuvStatusStagerNode)
