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
 * @file auv_status_bundler.cpp
 * @brief Implementation of the AuvStatusBundlerNode.
 * @author Nelson Durrant
 * @date June 2026
 */

#include "coug_comms/auv_status_bundler.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace coug_comms {

AuvStatusBundlerNode::AuvStatusBundlerNode(const rclcpp::NodeOptions& options)
    : Node("auv_status_bundler_node", options) {
  RCLCPP_INFO(get_logger(), "Starting AUV Status Bundler Node...");

  param_listener_ =
      std::make_shared<auv_status_bundler_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- ROS Interfaces ---
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.odom_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&AuvStatusBundlerNode::odomCallback, this, std::placeholders::_1));

  depth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.depth_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&AuvStatusBundlerNode::depthCallback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      params_.imu_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&AuvStatusBundlerNode::imuCallback, this, std::placeholders::_1));

  status_pub_ = create_publisher<coug_interfaces::msg::AgentStatus>(params_.status_topic,
                                                                    rclcpp::SystemDefaultsQoS());

  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / params_.publish_rate_hz),
                             std::bind(&AuvStatusBundlerNode::timerCallback, this));

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for sensor data...");
}

void AuvStatusBundlerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  last_odom_ = msg;
}

void AuvStatusBundlerNode::depthCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  last_depth_ = msg;
}

void AuvStatusBundlerNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  last_imu_ = msg;
}

void AuvStatusBundlerNode::timerCallback() {
  if (!last_odom_) {
    return;
  }

  coug_interfaces::msg::AgentStatus status;
  status.header.stamp = now();

  status.local_odometry = last_odom_->pose.pose;
  status.odometry_covariance = last_odom_->pose.covariance;

  if (last_depth_) {
    // TODO: Talk w Kalliyan -- which frame should everything be in?
    std::string depth_frame = last_depth_->child_frame_id;

    geometry_msgs::msg::TransformStamped depth_T_base_tf;
    try {
      depth_T_base_tf =
          tf_buffer_->lookupTransform(depth_frame, params_.base_frame, tf2::TimePointZero);

      geometry_msgs::msg::Pose p_base_in_depth;
      p_base_in_depth.position.x = depth_T_base_tf.transform.translation.x;
      p_base_in_depth.position.y = depth_T_base_tf.transform.translation.y;
      p_base_in_depth.position.z = depth_T_base_tf.transform.translation.z;
      p_base_in_depth.orientation = depth_T_base_tf.transform.rotation;

      geometry_msgs::msg::TransformStamped ref_T_depth_tf;
      ref_T_depth_tf.header.frame_id = last_depth_->header.frame_id;
      ref_T_depth_tf.child_frame_id = depth_frame;
      ref_T_depth_tf.transform.translation.x = last_depth_->pose.pose.position.x;
      ref_T_depth_tf.transform.translation.y = last_depth_->pose.pose.position.y;
      ref_T_depth_tf.transform.translation.z = last_depth_->pose.pose.position.z;
      ref_T_depth_tf.transform.rotation = last_depth_->pose.pose.orientation;

      geometry_msgs::msg::Pose p_base_in_ref;
      tf2::doTransform(p_base_in_depth, p_base_in_ref, ref_T_depth_tf);
      status.pressure_depth = p_base_in_ref.position.z;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Could not transform %s to %s: %s",
                           depth_frame.c_str(), params_.base_frame.c_str(), ex.what());
      status.pressure_depth = last_depth_->pose.pose.position.z;
    }
  } else {
    status.pressure_depth = 0.0;
  }

  if (last_imu_) {
    // Transform into the base_link frame
    std::string imu_frame = last_imu_->header.frame_id;

    geometry_msgs::msg::TransformStamped imu_T_base_tf;
    try {
      imu_T_base_tf =
          tf_buffer_->lookupTransform(imu_frame, params_.base_frame, tf2::TimePointZero);

      tf2::Quaternion q_ref_imu, q_imu_base;
      tf2::fromMsg(last_imu_->orientation, q_ref_imu);
      tf2::fromMsg(imu_T_base_tf.transform.rotation, q_imu_base);

      tf2::Quaternion q_ref_base = q_ref_imu * q_imu_base;
      q_ref_base.normalize();
      status.imu_orientation = tf2::toMsg(q_ref_base);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Could not transform %s to %s: %s",
                           imu_frame.c_str(), params_.base_frame.c_str(), ex.what());
      status.imu_orientation = last_imu_->orientation;
    }
  } else {
    status.imu_orientation.w = 1.0;
  }

  status_pub_->publish(status);
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::AuvStatusBundlerNode)
