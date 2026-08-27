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

#include "coug_comms/base_dispatcher.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include "coug_comms/utils/seatrac_enums.hpp"

namespace coug_comms {

using utils::CID_DAT_SEND;
using utils::MSG_OWAY;
using utils::MsgId;
using utils::ServiceOutcome;

namespace {

constexpr int kMaxBeaconId = 15;

std::string build_name(const std::string& agent, const std::string& sub) {
  return "/" + agent + "/" + sub;
}

}  // namespace

BaseDispatcherNode::BaseDispatcherNode(const rclcpp::NodeOptions& options)
    : Node("base_dispatcher_node", options), diagnostic_updater_(this) {
  param_listener_ =
      std::make_shared<base_dispatcher_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  modem_send_pub_ = create_publisher<seatrac_interfaces::msg::ModemSend>(
      params_.modem_send_topic, rclcpp::SystemDefaultsQoS());

  services_ = {
      {params_.start_service, params_.direct_start_service, MsgId::SRV_START},
      {params_.stop_service, params_.direct_stop_service, MsgId::SRV_STOP},
      {params_.surface_service, params_.direct_surface_service, MsgId::SRV_SURFACE},
      {params_.home_service, params_.direct_home_service, MsgId::SRV_HOME},
      {params_.emergency_stop_service, params_.direct_emergency_stop_service,
       MsgId::SRV_EMERGENCY_STOP},
      {params_.emergency_surface_service, params_.direct_emergency_surface_service,
       MsgId::SRV_EMERGENCY_SURFACE},
  };

  std::string prefix;
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/base_dispatcher_node");
    prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";
  }

  for (const auto& agent_name : params_.agent_namespaces) {
    int raw_id = this->declare_parameter<int>("beacon_ids." + agent_name, -1);
    if (raw_id < 0 || raw_id > kMaxBeaconId) {
      RCLCPP_ERROR(get_logger(), "Missing or invalid beacon_ids.%s (got %d) — skipping '%s'.",
                   agent_name.c_str(), raw_id, agent_name.c_str());
      continue;
    }
    registerAgent(agent_name, static_cast<uint8_t>(raw_id), prefix);
  }

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void BaseDispatcherNode::registerAgent(const std::string& agent_name, uint8_t beacon_id,
                                       const std::string& diag_prefix) {
  AgentEntry agent;
  agent.beacon_id = beacon_id;
  agent.is_lead = (agent_name == params_.lead_agent);

  for (const auto& spec : services_) {
    const MsgId cmd = spec.cmd;
    agent.services.push_back(create_service<std_srvs::srv::Trigger>(
        build_name(agent_name, spec.relay_service),
        [this, cmd, beacon_id](
            std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
            std::shared_ptr<rmw_request_id_t> header,
            std::shared_ptr<std_srvs::srv::Trigger::Request>) {
          handleServiceRequest(cmd, beacon_id, service_handle, header);
        }));
    agent.direct_clients[static_cast<uint8_t>(cmd)] =
        create_client<std_srvs::srv::Trigger>(build_name(agent_name, spec.direct_service));
  }

  if (params_.enable_direct_comms || agent.is_lead) {
    agent.direct_status_sub = create_subscription<coug_interfaces::msg::AgentStatus>(
        build_name(agent_name, params_.direct_status_topic), rclcpp::SystemDefaultsQoS(),
        [this, beacon_id](const coug_interfaces::msg::AgentStatus::SharedPtr) {
          auto it = agents_.find(beacon_id);
          if (it != agents_.end()) it->second.last_direct_heartbeat_sec = now().seconds();
        });
  }

  agents_.emplace(beacon_id, std::move(agent));

  if (params_.publish_diagnostics) {
    diagnostic_updater_.add(diag_prefix + "Service Status (" + agent_name + ")",
                            [this, beacon_id](diagnostic_updater::DiagnosticStatusWrapper& stat) {
                              checkAgentServiceStatus(stat, beacon_id);
                            });
  }

  RCLCPP_INFO(get_logger(), "Registered agent '%s' (beacon %d).", agent_name.c_str(), beacon_id);
}

void BaseDispatcherNode::handleServiceRequest(
    MsgId cmd, uint8_t beacon_id, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_handle,
    std::shared_ptr<rmw_request_id_t> header) {
  AgentEntry& agent = agents_.at(beacon_id);
  const std::string service = utils::toString(cmd);

  if (params_.enable_direct_comms || agent.is_lead) {
    if (directServiceDispatch(cmd, agent, service_handle, header)) {
      return;
    }
  }

  if (params_.enable_acoustic_comms && !agent.is_lead) {
    acousticServiceDispatch(cmd, agent, service_handle, header);
    return;
  }

  std_srvs::srv::Trigger::Response res;
  res.success = false;
  res.message = service + " failed: comms disabled.";
  service_handle->send_response(*header, res);
  RCLCPP_ERROR(get_logger(), "%s", res.message.c_str());
  recordServiceResult(beacon_id, service, "NONE", ServiceOutcome::FAILED);
}

bool BaseDispatcherNode::directServiceDispatch(
    MsgId cmd, const AgentEntry& agent,
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_handle,
    std::shared_ptr<rmw_request_id_t> header) {
  const bool direct_link_up =
      agent.last_direct_heartbeat_sec > 0.0 &&
      now().seconds() - agent.last_direct_heartbeat_sec < params_.direct_timeout_sec;
  auto client_it = agent.direct_clients.find(static_cast<uint8_t>(cmd));
  if (!direct_link_up || client_it == agent.direct_clients.end() ||
      !client_it->second->service_is_ready()) {
    return false;
  }

  const std::string service = utils::toString(cmd);
  const uint8_t beacon_id = agent.beacon_id;
  client_it->second->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this, service_handle, header, service,
       beacon_id](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        bool success = false;
        try {
          success = future.get()->success;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
        }
        std_srvs::srv::Trigger::Response res;
        res.success = success;
        res.message = service + (success ? " succeeded." : " failed.");
        service_handle->send_response(*header, res);
        if (success) {
          RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "%s", res.message.c_str());
        }
        recordServiceResult(beacon_id, service, "DIRECT",
                            success ? ServiceOutcome::SUCCEEDED : ServiceOutcome::FAILED);
      });
  return true;
}

void BaseDispatcherNode::acousticServiceDispatch(
    MsgId cmd, const AgentEntry& agent,
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_handle,
    std::shared_ptr<rmw_request_id_t> header) {
  seatrac_interfaces::msg::ModemSend msg;
  msg.msg_id = CID_DAT_SEND;
  msg.dest_id = agent.beacon_id;
  msg.msg_type = MSG_OWAY;
  msg.packet_len = 1;
  msg.packet_data[0] = static_cast<uint8_t>(cmd);
  modem_send_pub_->publish(msg);

  const std::string service = utils::toString(cmd);
  std_srvs::srv::Trigger::Response res;
  res.success = true;
  res.message = service + " queued (acomms).";
  service_handle->send_response(*header, res);
  RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
  recordServiceResult(agent.beacon_id, service, "ACOUSTIC", ServiceOutcome::QUEUED);
}

void BaseDispatcherNode::recordServiceResult(uint8_t beacon_id, const std::string& service,
                                             const std::string& transport, ServiceOutcome outcome) {
  AgentEntry& agent = agents_.at(beacon_id);
  agent.service_history.push_back({service, transport, outcome});
  if (agent.service_history.size() > static_cast<size_t>(params_.service_history_size)) {
    agent.service_history.pop_front();
  }
}

void BaseDispatcherNode::checkAgentServiceStatus(diagnostic_updater::DiagnosticStatusWrapper& stat,
                                                 uint8_t beacon_id) {
  const AgentEntry& agent = agents_.at(beacon_id);

  double direct_heartbeat_age = (agent.last_direct_heartbeat_sec > 0.0)
                                    ? (now().seconds() - agent.last_direct_heartbeat_sec)
                                    : -1.0;
  stat.add("Time Since Direct Heartbeat (s)", direct_heartbeat_age);

  if (agent.service_history.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Waiting for first service.");
    return;
  }

  std::string history_str;
  for (auto it = agent.service_history.rbegin(); it != agent.service_history.rend(); ++it) {
    history_str += "\n" + it->service + " (" + it->transport + "): " + utils::toString(it->outcome);
  }
  stat.add("Service History", history_str);

  const ServiceResult& latest = agent.service_history.back();
  const std::string summary = latest.service + " " + utils::toString(latest.outcome) + ".";
  if (latest.outcome == ServiceOutcome::FAILED) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, summary);
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, summary);
  }
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::BaseDispatcherNode)
