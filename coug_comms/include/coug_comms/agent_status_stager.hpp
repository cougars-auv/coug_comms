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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <seatrac_interfaces/msg/modem_send.hpp>

#include "coug_comms/agent_status_stager_parameters.hpp"
#include "coug_interfaces/msg/agent_status.hpp"

namespace coug_comms {

class AgentStatusStagerNode : public rclcpp::Node {
 public:
  explicit AgentStatusStagerNode(const rclcpp::NodeOptions& options);

 private:
  // --- Callbacks ---
  void statusCallback(const coug_interfaces::msg::AgentStatus::ConstSharedPtr& msg);

  // --- ROS Interfaces ---
  rclcpp::Subscription<coug_interfaces::msg::AgentStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_send_pub_;

  // --- Parameters ---
  std::shared_ptr<agent_status_stager_node::ParamListener> param_listener_;
  agent_status_stager_node::Params params_;
};

}  // namespace coug_comms
