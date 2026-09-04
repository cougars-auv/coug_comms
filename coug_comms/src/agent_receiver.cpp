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

#include "coug_comms/agent_receiver.hpp"

#include <cstddef>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <exception>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "coug_comms/agent_receiver_parameters.hpp"
#include "coug_comms/utils/protocol_enums.hpp"
#include "coug_comms/utils/service_enums.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace coug_comms {

using utils::MsgId;
using utils::ServiceOutcome;
using utils::toString;

AgentReceiverNode::AgentReceiverNode(const rclcpp::NodeOptions& options)
    : Node("agent_receiver_node", options), diagnostic_updater_(this) {
  param_listener_ =
      std::make_shared<agent_receiver_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  modem_rec_sub_ = create_subscription<seatrac_interfaces::msg::ModemRec>(
      params_.modem_rec_topic, rclcpp::SystemDefaultsQoS(),
      [this](const seatrac_interfaces::msg::ModemRec::ConstSharedPtr& msg) {
        modemRecCallback(msg);
      });

  start_client_ = create_client<std_srvs::srv::Trigger>(params_.start_service);
  stop_client_ = create_client<std_srvs::srv::Trigger>(params_.stop_service);
  surface_client_ = create_client<std_srvs::srv::Trigger>(params_.surface_service);
  home_client_ = create_client<std_srvs::srv::Trigger>(params_.home_service);
  emergency_stop_client_ = create_client<std_srvs::srv::Trigger>(params_.emergency_stop_service);
  emergency_surface_client_ =
      create_client<std_srvs::srv::Trigger>(params_.emergency_surface_service);

  if (params_.publish_diagnostics) {
    std::string const ns = this->get_namespace();
    std::string const clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/agent_receiver_node");

    std::string const prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";

    std::string const service_task = prefix + "Service Status";
    diagnostic_updater_.add(service_task, this, &AgentReceiverNode::checkServiceStatus);
  }

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void AgentReceiverNode::modemRecCallback(
    const seatrac_interfaces::msg::ModemRec::ConstSharedPtr& msg) {
  if (!msg->local_flag || msg->packet_len < 1) {
    return;
  }

  const auto msg_id = static_cast<MsgId>(msg->packet_data[0]);
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
  switch (msg_id) {
    case MsgId::kServiceStart:
      client = start_client_;
      break;
    case MsgId::kServiceStop:
      client = stop_client_;
      break;
    case MsgId::kServiceSurface:
      client = surface_client_;
      break;
    case MsgId::kServiceHome:
      client = home_client_;
      break;
    case MsgId::kEmergencyStop:
      client = emergency_stop_client_;
      break;
    case MsgId::kEmergencySurface:
      client = emergency_surface_client_;
      break;
    default:
      return;
  }

  RCLCPP_INFO(get_logger(), "Received %s from beacon %d.", toString(msg_id).c_str(), msg->src_id);
  callService(client, msg_id);
}

void AgentReceiverNode::callService(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
                                    MsgId msg) {
  const std::string service = toString(msg);
  if (!client->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Service not available: %s", service.c_str());
    recordServiceResult(service, "ACOUSTIC", ServiceOutcome::kFailed);
    return;
  }
  client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      // NOLINTNEXTLINE(performance-unnecessary-value-param)
      [this, service](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        bool success = false;
        try {
          success = future.get()->success;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Service call failed: %s; %s", service.c_str(), e.what());
        }
        recordServiceResult(service, "ACOUSTIC",
                            success ? ServiceOutcome::kSucceeded : ServiceOutcome::kFailed);
        if (success) {
          RCLCPP_INFO(get_logger(), "Service call succeeded: %s", service.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Service call failed: %s", service.c_str());
        }
      });
}

void AgentReceiverNode::recordServiceResult(const std::string& service,
                                            const std::string& transport, ServiceOutcome outcome) {
  service_history_.push_back({service, transport, outcome});
  if (service_history_.size() > static_cast<size_t>(params_.service_history_size)) {
    service_history_.pop_front();
  }
}

void AgentReceiverNode::checkServiceStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (service_history_.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Waiting for first service.");
    return;
  }

  std::string history_str;
  for (auto it = service_history_.rbegin(); it != service_history_.rend(); ++it) {
    history_str += "\n" + it->service + " (" + it->transport + "): " + toString(it->outcome);
  }
  stat.add("Service History", history_str);

  const ServiceResult& latest = service_history_.back();
  const std::string summary = latest.service + " " + toString(latest.outcome) + ".";
  if (latest.outcome == ServiceOutcome::kFailed) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, summary);
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, summary);
  }
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::AgentReceiverNode)
