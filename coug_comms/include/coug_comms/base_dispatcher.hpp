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

#include <deque>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <seatrac_interfaces/msg/modem_send.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "coug_comms/base_dispatcher_parameters.hpp"
#include "coug_comms/utils/protocol_enums.hpp"
#include "coug_interfaces/msg/agent_status.hpp"

namespace coug_comms {

class BaseDispatcherNode : public rclcpp::Node {
 public:
  explicit BaseDispatcherNode(const rclcpp::NodeOptions& options);

 private:
  struct ServiceSpec {
    std::string relay_service;
    std::string direct_service;
    utils::MsgId cmd;
  };

  struct ServiceResult {
    std::string service;
    std::string transport;
    bool succeeded;
  };

  struct AgentEntry {
    std::string name;
    uint8_t beacon_id;
    bool is_lead = false;
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    std::unordered_map<uint8_t, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> direct_clients;
    std::deque<ServiceResult> service_history;
    rclcpp::Subscription<coug_interfaces::msg::AgentStatus>::SharedPtr direct_heartbeat_sub;
    double last_direct_heartbeat_sec = 0.0;
  };

  // --- Helpers ---
  void registerAgent(const std::string& agent_name, uint8_t beacon_id,
                     const std::string& diag_prefix);

  void handleServiceRequest(utils::MsgId cmd, uint8_t beacon_id,
                            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
                            std::shared_ptr<rmw_request_id_t> header);

  bool directServiceDispatch(utils::MsgId cmd, const AgentEntry& agent,
                             rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
                             std::shared_ptr<rmw_request_id_t> header);

  void acousticServiceDispatch(utils::MsgId cmd, uint8_t beacon_id,
                               rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
                               std::shared_ptr<rmw_request_id_t> header);

  void recordServiceResult(uint8_t beacon_id, const std::string& service,
                           const std::string& transport, bool succeeded);

  // --- Diagnostics ---
  void checkAgentServiceStatus(diagnostic_updater::DiagnosticStatusWrapper& stat,
                               uint8_t beacon_id);

  // --- ROS Interfaces ---
  rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_send_pub_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<base_dispatcher_node::ParamListener> param_listener_;
  base_dispatcher_node::Params params_;

  // --- State ---
  std::vector<ServiceSpec> services_;
  std::unordered_map<uint8_t, AgentEntry> agents_;
};

}  // namespace coug_comms
