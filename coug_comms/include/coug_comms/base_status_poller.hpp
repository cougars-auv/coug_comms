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

#include <tf2_ros/transform_broadcaster.h>

#include <cstdint>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <seatrac_interfaces/msg/modem_cmd_update.hpp>
#include <seatrac_interfaces/msg/modem_rec.hpp>
#include <seatrac_interfaces/msg/modem_send.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "coug_comms/base_status_poller_parameters.hpp"
#include "coug_interfaces/msg/agent_status.hpp"

namespace coug_comms {

class BaseStatusPollerNode : public rclcpp::Node {
 public:
  explicit BaseStatusPollerNode(const rclcpp::NodeOptions& options);

 private:
  struct AgentEntry {
    std::string name;
    uint8_t beacon_id;
    bool is_lead = false;
    rclcpp::Publisher<coug_interfaces::msg::AgentStatus>::SharedPtr status_pub;
    rclcpp::Subscription<coug_interfaces::msg::AgentStatus>::SharedPtr direct_sub;
    size_t responses = 0;
    std::string last_transport;
    rclcpp::Time last_response_time;
    double last_direct_heartbeat_sec = 0.0;
  };

  // --- Callbacks ---
  void tickCallback();

  void modemRecCallback(const seatrac_interfaces::msg::ModemRec::SharedPtr msg);

  void modemCmdUpdateCallback(const seatrac_interfaces::msg::ModemCmdUpdate::SharedPtr msg);

  // --- Helpers ---
  void registerAgent(const std::string& agent_name, uint8_t beacon_id,
                     const std::string& diag_prefix);

  void pollNextIfReady();

  void scheduleNextPoll();

  void sendAcousticPoll(AgentEntry& agent);

  void publishStatus(AgentEntry& agent, coug_interfaces::msg::AgentStatus status,
                     const std::string& transport);

  void failPendingRequest(const char* reason);

  void publishPolledTransform(const AgentEntry& agent,
                              const seatrac_interfaces::msg::ModemRec& msg);

  // --- Diagnostics ---
  void checkAgentPollStatus(diagnostic_updater::DiagnosticStatusWrapper& stat, uint8_t beacon_id);

  // --- ROS Interfaces ---
  rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_rec_sub_;
  rclcpp::Subscription<seatrac_interfaces::msg::ModemCmdUpdate>::SharedPtr modem_cmd_update_sub_;
  rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_send_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  diagnostic_updater::Updater diagnostic_updater_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // --- Parameters ---
  std::shared_ptr<base_status_poller_node::ParamListener> param_listener_;
  base_status_poller_node::Params params_;

  // --- State ---
  std::vector<uint8_t> beacon_order_;
  std::unordered_map<uint8_t, AgentEntry> agents_;
  size_t next_beacon_idx_ = 0;
  bool awaiting_response_ = false;
  uint8_t pending_beacon_ = 0;
  rclcpp::Time request_time_;
  rclcpp::Time next_poll_allowed_;
};

}  // namespace coug_comms
