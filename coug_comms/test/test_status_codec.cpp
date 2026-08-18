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
 * @file test_status_codec.cpp
 * @brief Unit tests for status_codec.hpp.
 * @author Nelson Durrant (w Claude Opus 5)
 * @date June 2026
 */

#include <gtest/gtest.h>

#include <cmath>

#include "coug_comms/utils/status_codec.hpp"

namespace {

using coug_comms::utils::DatPayload;
using coug_comms::utils::decodeStatus;
using coug_comms::utils::encodeStatus;
using coug_comms::utils::kCovStride;
using coug_comms::utils::kStatusPacketLen;
using coug_comms::utils::detail::kMaxVariance;
using coug_comms::utils::detail::kMinVariance;

constexpr double kMetersTol = 0.005;    // half the 1 cm quantization step
constexpr double kQuatTol = 0.001;      // just over the 0.0007 component half-step
constexpr double kVarianceTol = 0.001;  // float16 keeps ~11 significant bits, so ~0.05% at worst

/**
 * @brief Builds a normalized quaternion.
 */
geometry_msgs::msg::Quaternion makeQuat(double x, double y, double z, double w) {
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  geometry_msgs::msg::Quaternion quat;
  quat.x = x / norm;
  quat.y = y / norm;
  quat.z = z / norm;
  quat.w = w / norm;
  return quat;
}

/**
 * @brief Checks two quaternions match within the component quantization, naming the call site.
 */
void expectQuatNear(const geometry_msgs::msg::Quaternion& actual,
                    const geometry_msgs::msg::Quaternion& expected, const char* label) {
  SCOPED_TRACE(label);
  EXPECT_NEAR(actual.x, expected.x, kQuatTol);
  EXPECT_NEAR(actual.y, expected.y, kQuatTol);
  EXPECT_NEAR(actual.z, expected.z, kQuatTol);
  EXPECT_NEAR(actual.w, expected.w, kQuatTol);
}

}  // namespace

/**
 * @brief Verify each transmitted field round-trips within its quantization or saturation bound.
 */
TEST(StatusCodecTest, RoundTrip) {
  coug_interfaces::msg::AgentStatus in;
  in.local_odometry.position.x = 12.34;
  in.local_odometry.position.y = -56.78;
  in.local_odometry.position.z = 3.20;
  in.pressure_depth = 4.05;
  in.local_odometry.orientation = makeQuat(-0.1, 0.2, -0.3, 0.9);
  in.imu_orientation = makeQuat(0.3, -0.4, 0.1, 0.8);
  for (int i = 0; i < 6; ++i) in.odometry_covariance[i * kCovStride] = 0.01 * (i + 1);
  in.odometry_covariance[0] = 1.0e-9;  // below kMinVariance, so the floor shows up
  in.odometry_covariance[35] = 1.0e6;  // above kMaxVariance, so the ceiling shows up

  DatPayload buf{};
  ASSERT_EQ(encodeStatus(in, buf), kStatusPacketLen);

  coug_interfaces::msg::AgentStatus out;
  out.header.frame_id = "coug1/base_link";
  out.odometry_covariance.fill(99.0);
  ASSERT_TRUE(decodeStatus(buf, kStatusPacketLen, out));

  EXPECT_NEAR(out.local_odometry.position.x, 12.34, kMetersTol);
  EXPECT_NEAR(out.local_odometry.position.y, -56.78, kMetersTol);
  EXPECT_NEAR(out.local_odometry.position.z, 3.20, kMetersTol);
  EXPECT_NEAR(out.pressure_depth, 4.05, kMetersTol);

  expectQuatNear(out.local_odometry.orientation, in.local_odometry.orientation, "local_odometry");
  expectQuatNear(out.imu_orientation, in.imu_orientation, "imu_orientation");

  EXPECT_NEAR(out.odometry_covariance[0], kMinVariance, kMinVariance * kVarianceTol);
  EXPECT_NEAR(out.odometry_covariance[35], kMaxVariance, kMaxVariance * kVarianceTol);
  for (int i = 1; i < 5; ++i) {
    const double expected = 0.01 * (i + 1);
    EXPECT_NEAR(out.odometry_covariance[i * kCovStride], expected, expected * kVarianceTol);
  }
  for (const int off_diagonal : {1, 6, 11, 34}) {
    EXPECT_DOUBLE_EQ(out.odometry_covariance[off_diagonal], 0.0) << "at " << off_diagonal;
  }

  EXPECT_EQ(out.header.frame_id, "coug1/base_link");  // the header is the caller's to fill in
}
