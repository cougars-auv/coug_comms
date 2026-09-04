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

#include "coug_comms/agent_status_bundler.hpp"

#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "coug_comms/agent_status_bundler_parameters.hpp"
#include "coug_interfaces/msg/agent_status.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace coug_comms {

using coug_interfaces::msg::AgentStatus;

AgentStatusBundlerNode::AgentStatusBundlerNode(const rclcpp::NodeOptions& options)
    : Node("agent_status_bundler_node", options) {
  param_listener_ =
      std::make_shared<agent_status_bundler_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.odom_topic, rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { odomCallback(msg); });

  depth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.depth_topic, rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { depthCallback(msg); });

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      params_.imu_topic, rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) { imuCallback(msg); });

  status_pub_ = create_publisher<AgentStatus>(params_.status_topic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void AgentStatusBundlerNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  last_odom_ = msg;
  publishStatus();
}

void AgentStatusBundlerNode::depthCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  last_depth_ = msg;
}

void AgentStatusBundlerNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  last_imu_ = msg;
}

void AgentStatusBundlerNode::publishStatus() {
  if (!last_odom_) {
    return;
  }

  AgentStatus status;
  status.header.stamp = now();

  status.local_odometry = last_odom_->pose.pose;
  status.odometry_covariance = last_odom_->pose.covariance;

  if (last_depth_) {
    // Transform depth data into the base frame
    std::string const depth_frame = last_depth_->child_frame_id;

    geometry_msgs::msg::TransformStamped depth_T_base_tf;
    try {
      depth_T_base_tf =
          tf_buffer_->lookupTransform(depth_frame, params_.base_frame, tf2::TimePointZero);

      geometry_msgs::msg::Pose depth_T_base;
      depth_T_base.position.x = depth_T_base_tf.transform.translation.x;
      depth_T_base.position.y = depth_T_base_tf.transform.translation.y;
      depth_T_base.position.z = depth_T_base_tf.transform.translation.z;
      depth_T_base.orientation = depth_T_base_tf.transform.rotation;

      geometry_msgs::msg::TransformStamped map_T_depth_tf;
      map_T_depth_tf.header.frame_id = last_depth_->header.frame_id;
      map_T_depth_tf.child_frame_id = depth_frame;
      map_T_depth_tf.transform.translation.x = last_depth_->pose.pose.position.x;
      map_T_depth_tf.transform.translation.y = last_depth_->pose.pose.position.y;
      map_T_depth_tf.transform.translation.z = last_depth_->pose.pose.position.z;

      tf2::Quaternion map_R_base;
      tf2::Quaternion depth_R_base;
      tf2::fromMsg(last_odom_->pose.pose.orientation, map_R_base);
      tf2::fromMsg(depth_T_base_tf.transform.rotation, depth_R_base);

      tf2::Quaternion map_R_depth = map_R_base * depth_R_base.inverse();
      map_R_depth.normalize();
      map_T_depth_tf.transform.rotation = tf2::toMsg(map_R_depth);

      geometry_msgs::msg::Pose map_T_base;
      tf2::doTransform(depth_T_base, map_T_base, map_T_depth_tf);
      status.pressure_depth = map_T_base.position.z;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Could not transform %s to %s: %s",
                           depth_frame.c_str(), params_.base_frame.c_str(), ex.what());
      status.pressure_depth = last_depth_->pose.pose.position.z;
    }
  } else {
    status.pressure_depth = 0.0;
  }

  if (last_imu_) {
    // Transform IMU data into the base frame
    std::string const imu_frame = last_imu_->header.frame_id;

    geometry_msgs::msg::TransformStamped imu_T_base_tf;
    try {
      imu_T_base_tf =
          tf_buffer_->lookupTransform(imu_frame, params_.base_frame, tf2::TimePointZero);

      tf2::Quaternion map_R_imu;
      tf2::Quaternion imu_R_base;
      tf2::fromMsg(last_imu_->orientation, map_R_imu);
      tf2::fromMsg(imu_T_base_tf.transform.rotation, imu_R_base);

      tf2::Quaternion map_R_base = map_R_imu * imu_R_base;
      map_R_base.normalize();
      status.imu_orientation = tf2::toMsg(map_R_base);
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

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::AgentStatusBundlerNode)
