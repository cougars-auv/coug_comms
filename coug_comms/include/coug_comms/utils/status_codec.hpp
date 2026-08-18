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
 * @file status_codec.hpp
 * @brief Acoustic encoding of coug_interfaces/AgentStatus.
 * @author Nelson Durrant (w Claude Opus 5)
 * @date June 2026
 */

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
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

// --- Field Codecs ---

/**
 * @brief Rounds a value to the nearest integer and clamps it into a range.
 * @param value The value to round (NaN is treated as zero).
 * @param lo The lowest allowed result.
 * @param hi The highest allowed result.
 * @return The rounded, clamped value.
 */
inline double roundClamp(double value, double lo, double hi) {
  if (std::isnan(value)) return 0.0;
  return std::clamp(std::round(value), lo, hi);
}

/**
 * @brief Encodes a distance as centimeter counts, saturating at roughly +/- 327 m.
 * @param meters The distance in meters.
 * @return The resulting distance in centimeters.
 */
inline int16_t encodeMeters(double meters) {
  return static_cast<int16_t>(roundClamp(meters * kCentimetersPerMeter,
                                         std::numeric_limits<int16_t>::lowest(),
                                         std::numeric_limits<int16_t>::max()));
}

/**
 * @brief Decodes centimeter counts back into meters.
 * @param counts The input distance in centimeters.
 * @return The resulting distance in meters.
 */
inline double decodeMeters(int16_t counts) {
  return static_cast<double>(counts) / kCentimetersPerMeter;
}

/**
 * @brief Clamps a variance into the representable range, mapping invalid values to the maximum.
 * @param variance The input variance.
 * @return The resulting finite, positive, encodable variance.
 */
inline double sanitizeVariance(double variance) {
  if (!std::isfinite(variance) || variance <= 0.0) return kMaxVariance;
  return std::clamp(variance, kMinVariance, kMaxVariance);
}

/**
 * @brief Encodes a variance as a float16, which keeps about three significant digits.
 * @param variance The variance to encode.
 * @return The resulting float16 bit pattern.
 */
inline uint16_t encodeVariance(double variance) {
  const auto half = Eigen::half(static_cast<float>(sanitizeVariance(variance)));
  return Eigen::numext::bit_cast<uint16_t>(half);
}

/**
 * @brief Decodes a float16 bit pattern back into a variance.
 * @param bits The input float16 bit pattern.
 * @return The resulting variance, re-clamped in case the bits arrived corrupted.
 */
inline double decodeVariance(uint16_t bits) {
  const auto half = Eigen::numext::bit_cast<Eigen::half>(bits);
  return sanitizeVariance(static_cast<double>(static_cast<float>(half)));
}

/**
 * @brief Packs a quaternion into 32 bits using smallest-three encoding.
 * @param quat The quaternion to encode (need not be normalized).
 * @return The resulting packed quaternion.
 */
inline uint32_t encodeQuaternion(const geometry_msgs::msg::Quaternion& quat) {
  double q[4] = {quat.x, quat.y, quat.z, quat.w};
  double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!std::isfinite(norm) || norm < 1.0e-9) {
    q[0] = q[1] = q[2] = 0.0;
    q[3] = norm = 1.0;
  }

  int largest = 0;
  for (int i = 1; i < 4; ++i) {
    if (std::fabs(q[i]) > std::fabs(q[largest])) largest = i;
  }
  const double sign = (q[largest] < 0.0) ? -1.0 : 1.0;
  for (double& component : q) component = component * sign / norm;

  uint32_t packed = static_cast<uint32_t>(largest) << kQuatSelectorShift;
  for (int i = 0, shift = kQuatSelectorShift; i < 4; ++i) {
    if (i == largest) continue;
    shift -= kQuatBits;
    const auto counts =
        static_cast<int32_t>(roundClamp(q[i] / kQuatLimit * kQuatMax, -kQuatMax, kQuatMax));
    packed |= (static_cast<uint32_t>(counts) & (kQuatRange - 1)) << shift;
  }
  return packed;
}

/**
 * @brief Unpacks a smallest-three encoded quaternion.
 * @param packed The input packed quaternion.
 * @return The resulting normalized quaternion.
 */
inline geometry_msgs::msg::Quaternion decodeQuaternion(uint32_t packed) {
  const int largest = static_cast<int>(packed >> kQuatSelectorShift);

  double q[4];
  double sum_squares = 0.0;
  for (int i = 0, shift = kQuatSelectorShift; i < 4; ++i) {
    if (i == largest) continue;
    shift -= kQuatBits;
    int32_t counts = static_cast<int32_t>((packed >> shift) & (kQuatRange - 1));
    if (counts > kQuatMax) counts -= kQuatRange;  // Sign extend the 10-bit field
    q[i] = static_cast<double>(counts) / kQuatMax * kQuatLimit;
    sum_squares += q[i] * q[i];
  }
  q[largest] = std::sqrt(std::max(0.0, 1.0 - sum_squares));

  // Already unit length unless the bits arrived corrupted and pushed the sum past one
  const double norm = std::sqrt(sum_squares + q[largest] * q[largest]);

  geometry_msgs::msg::Quaternion quat;
  quat.x = q[0] / norm;
  quat.y = q[1] / norm;
  quat.z = q[2] / norm;
  quat.w = q[3] / norm;
  return quat;
}

// --- Payload Cursor ---

/**
 * @class PayloadCursor
 * @brief Walks a DAT payload, reading or writing fields in order.
 */
class PayloadCursor {
 public:
  /**
   * @brief Writes a value at the cursor and advances past it.
   * @tparam T The trivially-copyable field type.
   * @param buf The payload to write into.
   * @param value The value to write.
   */
  template <typename T>
  void put(DatPayload& buf, T value) {
    assert(off_ + sizeof(T) <= buf.size());
    std::memcpy(buf.data() + off_, &value, sizeof(T));
    off_ += sizeof(T);
  }

  /**
   * @brief Reads a value at the cursor and advances past it.
   * @tparam T The trivially-copyable field type.
   * @param buf The payload to read from.
   * @return The value read.
   */
  template <typename T>
  T get(const DatPayload& buf) {
    assert(off_ + sizeof(T) <= buf.size());
    T value;
    std::memcpy(&value, buf.data() + off_, sizeof(T));
    off_ += sizeof(T);
    return value;
  }

  /**
   * @brief Gets the number of bytes the cursor has covered.
   * @return The current offset into the payload.
   */
  uint8_t offset() const { return static_cast<uint8_t>(off_); }

 private:
  size_t off_ = 0;
};

// --- Status Codec ---

/**
 * @brief Encodes an AgentStatus into an acoustic DAT payload.
 * @param status The status to encode.
 * @param buf The payload to write into.
 * @return The number of bytes written (always kStatusPacketLen).
 */
inline uint8_t encodeStatus(const coug_interfaces::msg::AgentStatus& status, DatPayload& buf) {
  PayloadCursor cursor;
  cursor.put<uint8_t>(buf, static_cast<uint8_t>(MsgId::RESP_STATUS));

  const auto& pose = status.local_odometry;
  cursor.put(buf, encodeMeters(pose.position.x));
  cursor.put(buf, encodeMeters(pose.position.y));
  cursor.put(buf, encodeMeters(pose.position.z));
  cursor.put(buf, encodeQuaternion(pose.orientation));

  for (int i = 0; i < kCovDim; ++i) {
    cursor.put(buf, encodeVariance(status.odometry_covariance[i * kCovStride]));
  }

  cursor.put(buf, encodeMeters(status.pressure_depth));
  cursor.put(buf, encodeQuaternion(status.imu_orientation));

  assert(cursor.offset() == kStatusPacketLen);
  return cursor.offset();
}

/**
 * @brief Decodes an acoustic DAT payload, zeroing the covariance terms it does not carry.
 * @param buf The received payload.
 * @param len The received payload length.
 * @param status The status to populate.
 * @return True if the payload was long enough and carried a RESP_STATUS id, false otherwise.
 */
inline bool decodeStatus(const DatPayload& buf, uint8_t len,
                         coug_interfaces::msg::AgentStatus& status) {
  if (len < kStatusPacketLen) return false;
  PayloadCursor cursor;
  if (cursor.get<uint8_t>(buf) != static_cast<uint8_t>(MsgId::RESP_STATUS)) return false;

  auto& pose = status.local_odometry;
  pose.position.x = decodeMeters(cursor.get<int16_t>(buf));
  pose.position.y = decodeMeters(cursor.get<int16_t>(buf));
  pose.position.z = decodeMeters(cursor.get<int16_t>(buf));
  pose.orientation = decodeQuaternion(cursor.get<uint32_t>(buf));

  status.odometry_covariance.fill(0.0);
  for (int i = 0; i < kCovDim; ++i) {
    status.odometry_covariance[i * kCovStride] = decodeVariance(cursor.get<uint16_t>(buf));
  }

  status.pressure_depth = decodeMeters(cursor.get<int16_t>(buf));
  status.imu_orientation = decodeQuaternion(cursor.get<uint32_t>(buf));
  return true;
}

}  // namespace coug_comms::utils
