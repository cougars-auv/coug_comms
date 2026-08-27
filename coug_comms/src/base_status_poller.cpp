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

#include "coug_comms/base_status_poller.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "coug_comms/utils/protocol_enums.hpp"
#include "coug_comms/utils/seatrac_enums.hpp"
#include "coug_comms/utils/status_codec.hpp"

namespace coug_comms {

using utils::CID_DAT_SEND;
using utils::CST_XCVR_RESP_TIMEOUT;
using utils::MSG_REQX;
using utils::MsgId;

namespace {

constexpr double kDecimetersToMeters = 0.1;
constexpr double kSeatracToRad = M_PI / 1800.0;
constexpr int kMaxBeaconId = 15;

std::string build_name(const std::string& agent, const std::string& sub) {
  return "/" + agent + "/" + sub;
}

}  // namespace

BaseStatusPollerNode::BaseStatusPollerNode(const rclcpp::NodeOptions& options)
    : Node("base_status_poller_node", options), diagnostic_updater_(this) {
  param_listener_ =
      std::make_shared<base_status_poller_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  modem_rec_sub_ = create_subscription<seatrac_interfaces::msg::ModemRec>(
      params_.modem_rec_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BaseStatusPollerNode::modemRecCallback, this, std::placeholders::_1));

  modem_cmd_update_sub_ = create_subscription<seatrac_interfaces::msg::ModemCmdUpdate>(
      params_.modem_cmd_update_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BaseStatusPollerNode::modemCmdUpdateCallback, this, std::placeholders::_1));

  modem_send_pub_ = create_publisher<seatrac_interfaces::msg::ModemSend>(
      params_.modem_send_topic, rclcpp::SystemDefaultsQoS());

  std::string prefix;
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/base_status_poller_node");
    prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";
  }

  for (const auto& agent_name : params_.agent_list) {
    int raw_id = this->declare_parameter<int>("beacon_ids." + agent_name, -1);
    if (raw_id < 0 || raw_id > kMaxBeaconId) {
      RCLCPP_ERROR(get_logger(), "Missing or invalid beacon_ids.%s (got %d) — skipping '%s'.",
                   agent_name.c_str(), raw_id, agent_name.c_str());
      continue;
    }
    registerAgent(agent_name, static_cast<uint8_t>(raw_id), prefix);
  }

  next_poll_allowed_ = now();
  tick_timer_ = create_wall_timer(std::chrono::duration<double>(params_.tick_period_sec),
                                  std::bind(&BaseStatusPollerNode::tickCallback, this));

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void BaseStatusPollerNode::tickCallback() {
  if (awaiting_response_ && (now() - request_time_).seconds() > params_.response_timeout_sec) {
    failPendingRequest("missed driver report, node-level response timeout");
    return;
  }
  pollNextIfReady();
}

void BaseStatusPollerNode::modemRecCallback(
    const seatrac_interfaces::msg::ModemRec::SharedPtr msg) {
  if (!awaiting_response_ || !msg->local_flag || msg->src_id != pending_beacon_) return;

  auto it = agents_.find(pending_beacon_);
  if (it == agents_.end()) {
    failPendingRequest("no agent registered for the pending beacon");
    return;
  }

  coug_interfaces::msg::AgentStatus status;
  if (!utils::decodeStatus(msg->packet_data, msg->packet_len, status)) {
    failPendingRequest("undecodable status payload");
    return;
  }

  status.includes_range = msg->includes_range;
  status.range_dist = msg->includes_range ? msg->range_dist * kDecimetersToMeters : 0.0;

  // Convert FRD -> FLU
  status.includes_usbl = msg->includes_usbl;
  status.usbl_azimuth = msg->includes_usbl ? -msg->usbl_azimuth * kSeatracToRad : 0.0;
  status.usbl_elevation = msg->includes_usbl ? msg->usbl_elevation * kSeatracToRad : 0.0;

  status.includes_position = msg->includes_position;
  status.position_depth = msg->includes_position ? msg->position_depth * kDecimetersToMeters : 0.0;

  status.header.frame_id =
      params_.use_parameter_frame ? params_.parameter_frame : msg->header.frame_id;

  publishStatus(it->second, status, "ACOUSTIC");
  publishPolledTransform(it->second, *msg);

  finishPendingRequest();
}

void BaseStatusPollerNode::modemCmdUpdateCallback(
    const seatrac_interfaces::msg::ModemCmdUpdate::SharedPtr msg) {
  if (!awaiting_response_ || msg->target_id != pending_beacon_) return;
  if (msg->command_status_code != CST_XCVR_RESP_TIMEOUT) return;

  failPendingRequest("driver-level response timeout");
}

void BaseStatusPollerNode::registerAgent(const std::string& agent_name, uint8_t beacon_id,
                                         const std::string& diag_prefix) {
  AgentEntry agent;
  agent.name = agent_name;
  agent.beacon_id = beacon_id;
  agent.is_lead = (agent_name == params_.lead_agent);
  agent.status_pub = create_publisher<coug_interfaces::msg::AgentStatus>(
      build_name(agent_name, params_.status_topic), rclcpp::SystemDefaultsQoS());
  agent.last_response_time = now();

  if (params_.enable_direct_comms || agent.is_lead) {
    agent.direct_status_sub = create_subscription<coug_interfaces::msg::AgentStatus>(
        build_name(agent_name, params_.direct_status_topic), rclcpp::SystemDefaultsQoS(),
        [this, beacon_id](const coug_interfaces::msg::AgentStatus::SharedPtr msg) {
          auto it = agents_.find(beacon_id);
          if (it == agents_.end()) return;
          it->second.last_direct_heartbeat_sec = now().seconds();
          publishStatus(it->second, *msg, "DIRECT");
        });
  }

  if (!agent.is_lead) {
    beacon_order_.push_back(beacon_id);
  }
  agents_.emplace(beacon_id, std::move(agent));

  if (params_.publish_diagnostics) {
    diagnostic_updater_.add(diag_prefix + "Polling Status (" + agent_name + ")",
                            [this, beacon_id](diagnostic_updater::DiagnosticStatusWrapper& stat) {
                              checkAgentPollStatus(stat, beacon_id);
                            });
  }

  RCLCPP_INFO(get_logger(), "Registered agent '%s' (beacon %d).", agent_name.c_str(), beacon_id);
}

void BaseStatusPollerNode::pollNextIfReady() {
  if (awaiting_response_ || beacon_order_.empty()) return;
  if (now() < next_poll_allowed_) return;

  AgentEntry& agent = agents_.at(beacon_order_[next_beacon_idx_]);
  next_beacon_idx_ = (next_beacon_idx_ + 1) % beacon_order_.size();

  const bool direct_link_up =
      agent.last_direct_heartbeat_sec > 0.0 &&
      now().seconds() - agent.last_direct_heartbeat_sec < params_.direct_timeout_sec;
  if (params_.enable_direct_comms && direct_link_up) {
    scheduleNextPoll();
    return;
  }
  if (params_.enable_acoustic_comms) {
    sendAcousticPoll(agent);
    return;
  }
  scheduleNextPoll();
}

void BaseStatusPollerNode::scheduleNextPoll() {
  next_poll_allowed_ = now() + rclcpp::Duration::from_seconds(params_.poll_period_sec);
}

void BaseStatusPollerNode::sendAcousticPoll(AgentEntry& agent) {
  seatrac_interfaces::msg::ModemSend send_msg;
  send_msg.msg_id = CID_DAT_SEND;
  send_msg.dest_id = agent.beacon_id;
  send_msg.msg_type = MSG_REQX;
  send_msg.packet_len = 1;
  send_msg.packet_data[0] = static_cast<uint8_t>(MsgId::kStatusRequest);
  modem_send_pub_->publish(send_msg);

  awaiting_response_ = true;
  pending_beacon_ = agent.beacon_id;
  request_time_ = now();
}

void BaseStatusPollerNode::publishStatus(AgentEntry& agent,
                                         coug_interfaces::msg::AgentStatus status,
                                         const std::string& transport) {
  status.header.stamp = now();
  agent.status_pub->publish(status);

  agent.responses++;
  agent.last_transport = transport;
  agent.last_response_time = now();
}

void BaseStatusPollerNode::finishPendingRequest() {
  awaiting_response_ = false;
  scheduleNextPoll();
  pollNextIfReady();
}

void BaseStatusPollerNode::failPendingRequest(const std::string& reason) {
  RCLCPP_WARN(get_logger(), "Beacon %d: %s.", pending_beacon_, reason.c_str());
  finishPendingRequest();
}

void BaseStatusPollerNode::publishPolledTransform(const AgentEntry& agent,
                                                  const seatrac_interfaces::msg::ModemRec& msg) {
  if (!msg.includes_usbl || !msg.includes_range || !msg.includes_position) {
    return;
  }

  // Convert FRD -> FLU
  const double azimuth = -msg.usbl_azimuth * kSeatracToRad;
  const double range = msg.range_dist * kDecimetersToMeters;
  const double depth = (msg.position_depth - msg.depth_local) * kDecimetersToMeters;
  const double horizontal_range = std::sqrt(std::max(range * range - depth * depth, 0.0));

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = now();
  tf_msg.header.frame_id =
      params_.use_parameter_frame ? params_.parameter_frame : msg.header.frame_id;
  tf_msg.child_frame_id = agent.name + "/polled_modem_link";
  tf_msg.transform.translation.x = horizontal_range * std::cos(azimuth);
  tf_msg.transform.translation.y = horizontal_range * std::sin(azimuth);
  tf_msg.transform.translation.z = -depth;
  tf_msg.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(tf_msg);
}

void BaseStatusPollerNode::checkAgentPollStatus(diagnostic_updater::DiagnosticStatusWrapper& stat,
                                                uint8_t beacon_id) {
  const AgentEntry& agent = agents_.at(beacon_id);

  double direct_heartbeat_age = (agent.last_direct_heartbeat_sec > 0.0)
                                    ? (now().seconds() - agent.last_direct_heartbeat_sec)
                                    : -1.0;
  stat.add("Time Since Direct Heartbeat (s)", direct_heartbeat_age);

  double time_since = (agent.responses > 0) ? (now() - agent.last_response_time).seconds() : -1.0;
  if (agent.responses > 0) stat.add("Last Transport", agent.last_transport);
  stat.add("Time Since Last (s)", time_since);

  if (agent.responses == 0 || time_since > params_.diagnostic_timeout_sec) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Agent is unreachable.");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Polled status acquired.");
  }
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::BaseStatusPollerNode)
