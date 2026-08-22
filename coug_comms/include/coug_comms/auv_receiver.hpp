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
#include <seatrac_interfaces/msg/modem_rec.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "coug_comms/auv_receiver_parameters.hpp"
#include "coug_comms/utils/protocol_enums.hpp"

namespace coug_comms {

class AuvReceiverNode : public rclcpp::Node {
 public:
  explicit AuvReceiverNode(const rclcpp::NodeOptions& options);

 private:
  void modemRecCallback(const seatrac_interfaces::msg::ModemRec::SharedPtr msg);

  void callService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, utils::MsgId cmd);

  void recordServiceResult(const std::string& service, const std::string& transport,
                           bool succeeded);

  void checkServiceStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // --- ROS Interfaces ---
  rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_rec_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr surface_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr home_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_surface_client_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<auv_receiver_node::ParamListener> param_listener_;
  auv_receiver_node::Params params_;

  // --- State ---
  struct ServiceResult {
    std::string service;
    std::string transport;
    bool succeeded;
  };
  std::deque<ServiceResult> service_history_;
};

}  // namespace coug_comms
