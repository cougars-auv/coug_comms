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

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

#include "coug_comms/utils/protocol_enums.hpp"
#include "coug_interfaces/msg/agent_status.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace coug_comms::utils {

using DatPayload = std::array<uint8_t, 30>;

inline constexpr int kCovDim = 6;
inline constexpr int kCovStride = kCovDim + 1;

// Byte length of an encoded status packet
inline constexpr uint8_t kStatusPacketLen =
    static_cast<uint8_t>(sizeof(uint8_t) + 3 * sizeof(int16_t) + sizeof(uint32_t) +
                         kCovDim * sizeof(uint16_t) + sizeof(int16_t) + sizeof(uint32_t));

static_assert(kStatusPacketLen <= std::tuple_size<DatPayload>::value,
              "An encoded status must fit in one acoustic DAT payload");

inline constexpr double kCentimetersPerMeter = 100.0;

// Bounds keeping variances inside the range float16 represents with useful precision
inline constexpr double kMinVariance = 1.0e-4;
inline constexpr double kMaxVariance = 6.0e4;

// Smallest-three packing: a 2-bit selector for the dropped component + three signed 10-bit counts
inline constexpr int kQuatBits = 10;
inline constexpr int kQuatSelectorShift = 3 * kQuatBits;
inline constexpr int32_t kQuatRange = 1 << kQuatBits;
inline constexpr int32_t kQuatMax = kQuatRange / 2 - 1;
inline constexpr double kQuatLimit = M_SQRT1_2;

inline auto roundClamp(double value, double min_value, double max_value) -> double {
  if (std::isnan(value)) {
    return 0.0;
  }
  return std::clamp(std::round(value), min_value, max_value);
}

inline auto encodeMeters(double meters) -> int16_t {
  return static_cast<int16_t>(roundClamp(meters * kCentimetersPerMeter,
                                         std::numeric_limits<int16_t>::lowest(),
                                         std::numeric_limits<int16_t>::max()));
}

inline auto decodeMeters(int16_t counts) -> double {
  return static_cast<double>(counts) / kCentimetersPerMeter;
}

inline auto sanitizeVariance(double variance) -> double {
  if (!std::isfinite(variance) || variance <= 0.0) {
    return kMaxVariance;
  }
  return std::clamp(variance, kMinVariance, kMaxVariance);
}

inline auto encodeVariance(double variance) -> uint16_t {
  const auto half = Eigen::half(static_cast<float>(sanitizeVariance(variance)));
  return Eigen::numext::bit_cast<uint16_t>(half);
}

inline auto decodeVariance(uint16_t bits) -> double {
  const auto half = Eigen::numext::bit_cast<Eigen::half>(bits);
  return sanitizeVariance(static_cast<double>(static_cast<float>(half)));
}

inline auto encodeQuaternion(const geometry_msgs::msg::Quaternion& q) -> uint32_t {
  std::array<double, 4> q_vec = {q.x, q.y, q.z, q.w};
  double norm = std::sqrt(q_vec[0] * q_vec[0] + q_vec[1] * q_vec[1] + q_vec[2] * q_vec[2] +
                          q_vec[3] * q_vec[3]);
  if (!std::isfinite(norm) || norm < 1.0e-9) {
    q_vec[0] = q_vec[1] = q_vec[2] = 0.0;
    q_vec[3] = norm = 1.0;
  }

  int largest_idx = 0;
  for (int i = 1; i < 4; ++i) {
    if (std::fabs(q_vec[i]) > std::fabs(q_vec[largest_idx])) {
      largest_idx = i;
    }
  }
  const double sign = (q_vec[largest_idx] < 0.0) ? -1.0 : 1.0;
  for (double& component : q_vec) {
    component = component * sign / norm;
  }

  uint32_t packed = static_cast<uint32_t>(largest_idx) << kQuatSelectorShift;
  for (int i = 0, shift = kQuatSelectorShift; i < 4; ++i) {
    if (i == largest_idx) {
      continue;
    }
    shift -= kQuatBits;
    const auto counts =
        static_cast<int32_t>(roundClamp(q_vec[i] / kQuatLimit * kQuatMax, -kQuatMax, kQuatMax));
    packed |= (static_cast<uint32_t>(counts) & (kQuatRange - 1)) << shift;
  }
  return packed;
}

inline auto decodeQuaternion(uint32_t packed) -> geometry_msgs::msg::Quaternion {
  const int largest_idx = static_cast<int>(packed >> kQuatSelectorShift);

  std::array<double, 4> q_vec{};
  double sum_squares = 0.0;
  for (int i = 0, shift = kQuatSelectorShift; i < 4; ++i) {
    if (i == largest_idx) {
      continue;
    }
    shift -= kQuatBits;
    auto counts = static_cast<int32_t>((packed >> shift) & (kQuatRange - 1));
    if (counts > kQuatMax) {  // Sign extend the 10-bit field
      counts -= kQuatRange;
    }
    q_vec[i] = static_cast<double>(counts) / kQuatMax * kQuatLimit;
    sum_squares += q_vec[i] * q_vec[i];
  }
  q_vec[largest_idx] = std::sqrt(std::max(0.0, 1.0 - sum_squares));

  // Already unit length unless the bits arrived corrupted and pushed the sum past one
  const double norm = std::sqrt(sum_squares + q_vec[largest_idx] * q_vec[largest_idx]);

  geometry_msgs::msg::Quaternion q;
  q.x = q_vec[0] / norm;
  q.y = q_vec[1] / norm;
  q.z = q_vec[2] / norm;
  q.w = q_vec[3] / norm;
  return q;
}

class PayloadCursor {
 public:
  template <typename T>
  void put(DatPayload& payload, T value) {
    assert(offset_ + sizeof(T) <= payload.size());
    std::memcpy(payload.data() + offset_, &value, sizeof(T));
    offset_ += sizeof(T);
  }

  template <typename T>
  auto get(const DatPayload& payload) -> T {
    assert(offset_ + sizeof(T) <= payload.size());
    T value;
    std::memcpy(&value, payload.data() + offset_, sizeof(T));
    offset_ += sizeof(T);
    return value;
  }

  [[nodiscard]] auto offset() const -> uint8_t { return static_cast<uint8_t>(offset_); }

 private:
  size_t offset_ = 0;
};

inline auto encodeStatus(const coug_interfaces::msg::AgentStatus& status, DatPayload& payload)
    -> uint8_t {
  PayloadCursor cursor;
  cursor.put<uint8_t>(payload, static_cast<uint8_t>(MsgId::kStatusResponse));

  const auto& pose = status.local_odometry;
  cursor.put(payload, encodeMeters(pose.position.x));
  cursor.put(payload, encodeMeters(pose.position.y));
  cursor.put(payload, encodeMeters(pose.position.z));
  cursor.put(payload, encodeQuaternion(pose.orientation));

  for (int i = 0; i < kCovDim; ++i) {
    cursor.put(payload, encodeVariance(
                            status.odometry_covariance[static_cast<std::size_t>(i) * kCovStride]));
  }

  cursor.put(payload, encodeMeters(status.pressure_depth));
  cursor.put(payload, encodeQuaternion(status.imu_orientation));

  assert(cursor.offset() == kStatusPacketLen);
  return cursor.offset();
}

inline auto decodeStatus(const DatPayload& payload, uint8_t packet_len,
                         coug_interfaces::msg::AgentStatus& status) -> bool {
  if (packet_len < kStatusPacketLen) {
    return false;
  }
  PayloadCursor cursor;
  if (cursor.get<uint8_t>(payload) != static_cast<uint8_t>(MsgId::kStatusResponse)) {
    return false;
  }

  auto& pose = status.local_odometry;
  pose.position.x = decodeMeters(cursor.get<int16_t>(payload));
  pose.position.y = decodeMeters(cursor.get<int16_t>(payload));
  pose.position.z = decodeMeters(cursor.get<int16_t>(payload));
  pose.orientation = decodeQuaternion(cursor.get<uint32_t>(payload));

  status.odometry_covariance.fill(0.0);
  for (int i = 0; i < kCovDim; ++i) {
    status.odometry_covariance[static_cast<std::size_t>(i) * kCovStride] =
        decodeVariance(cursor.get<uint16_t>(payload));
  }

  status.pressure_depth = decodeMeters(cursor.get<int16_t>(payload));
  status.imu_orientation = decodeQuaternion(cursor.get<uint32_t>(payload));
  return true;
}

}  // namespace coug_comms::utils
