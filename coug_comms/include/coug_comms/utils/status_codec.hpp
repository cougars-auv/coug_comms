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
 *
 * AgentStatus is too large for a Seatrac DAT packet (max 30 bytes), so only the
 * navigation summary is quantized onto the wire. The 29-byte layout:
 *
 *   offset  field                              encoding
 *   ------  ---------------------------------  -----------------------------
 *    0      MsgId::RESP_STATUS discriminator    uint8
 *    1- 6   local_odometry.position (x,y,z)    int16 x3, 1 cm (+/- 327.67 m)
 *    7-10   local_odometry.orientation         smallest-three quaternion
 *   11-22   odometry_covariance diagonal (6)   float16 x6, bounded [1e-6, 6e4]
 *   23-24   pressure_depth                     int16, 1 cm
 *   25-28   imu_orientation                    smallest-three quaternion
 *
 * The header and off-diagonal covariance are not sent; multi-byte fields are
 * little-endian. Every codec below is total: any input, including NaN and
 * arbitrary wire bits, decodes to something finite and in range, since both
 * directions feed a factor graph that a single NaN poisons.
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

// --- Wire Layout ---

inline constexpr size_t kMaxDatPayloadLen = 30;  // matches the ModemSend/ModemRec buffers
inline constexpr int kCovDim = 6;                // AgentStatus carries the 6x6 row-major
inline constexpr int kCovStride = kCovDim + 1;   // step between diagonal entries

using DatPayload = std::array<uint8_t, kMaxDatPayloadLen>;

// Summed in wire order, so the length follows when a field's encoding changes
inline constexpr uint8_t kStatusPacketLen =
    static_cast<uint8_t>(sizeof(uint8_t) +             // MsgId discriminator
                         3 * sizeof(int16_t) +         // position
                         sizeof(uint32_t) +            // orientation
                         kCovDim * sizeof(uint16_t) +  // covariance diagonal
                         sizeof(int16_t) +             // pressure depth
                         sizeof(uint32_t));            // imu orientation

static_assert(kStatusPacketLen == 29,
              "wire layout changed; update the byte table in the file comment above");
static_assert(kStatusPacketLen <= kMaxDatPayloadLen, "status packet does not fit a DAT payload");
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
              "wire format assumes a little-endian host");

namespace detail {

// --- Quantization ---

/**
 * @brief Rounds to the nearest integer and saturates into [lo, hi], mapping NaN to zero.
 */
inline double roundSaturating(double value, double lo, double hi) {
  // Clamping in double space keeps an infinity on the saturating path, whether it arrived that
  // way or the scaling overflowed into it; std::lround() would instead hit a domain error
  if (std::isnan(value)) return 0.0;
  return std::clamp(std::round(value), lo, hi);
}

/**
 * @brief Rounds to the nearest T and saturates into T's full range, mapping NaN to zero.
 */
template <typename T>
inline T roundSaturating(double value) {
  return static_cast<T>(roundSaturating(value,
                                        static_cast<double>(std::numeric_limits<T>::lowest()),
                                        static_cast<double>(std::numeric_limits<T>::max())));
}

// --- Distance Codec ---

inline constexpr double kCentimetersPerMeter = 100.0;

/**
 * @brief Quantizes meters to 1 cm signed fixed point, saturating past +/- 327.67 m.
 */
inline int16_t encodeMeters(double meters) {
  return roundSaturating<int16_t>(meters * kCentimetersPerMeter);
}

/**
 * @brief Restores meters from the 1 cm fixed-point encoding.
 */
inline double decodeMeters(int16_t counts) {
  return static_cast<double>(counts) / kCentimetersPerMeter;
}

// --- Variance Codec ---

// IMPORTANT! Keep the covariance diagonal inside what a float16 can represent
//
// Half precision flushes below ~6e-8 to zero and saturates above 65504 to infinity, and a receiver
// cannot tell either from a real value: a near-zero variance is an infinitely confident neighbor
// that makes the downstream linear system singular, and a non-finite one gets the whole keyframe
// thrown away. Both directions bound their input, costing a saturated bound instead of a message.
inline constexpr double kMinVariance = 1.0e-4;  // 1 cm sigma, the wire's own position resolution
inline constexpr double kMaxVariance = 6.0e4;   // 245 m sigma, and 172 steps below half's max

/**
 * @brief Bounds a variance into the float16 range, mapping anything unusable to kMaxVariance.
 */
inline double sanitizeVariance(double variance) {
  // std::clamp() alone would not do: NaN compares false against both ends and passes straight
  // through, and a garbage non-positive value rounds *up* into millimeter confidence
  if (!std::isfinite(variance) || variance <= 0.0) return kMaxVariance;
  return std::clamp(variance, kMinVariance, kMaxVariance);
}

/**
 * @brief Bounds a variance and packs it into half-precision bits.
 */
inline uint16_t encodeVariance(double variance) {
  const auto half = Eigen::half(static_cast<float>(sanitizeVariance(variance)));
  return Eigen::numext::bit_cast<uint16_t>(half);
}

/**
 * @brief Unpacks a half-precision variance and re-bounds it.
 */
inline double decodeVariance(uint16_t bits) {
  const auto half = Eigen::numext::bit_cast<Eigen::half>(bits);
  return sanitizeVariance(static_cast<double>(static_cast<float>(half)));
}

// --- Rotation Codec ---

// A unit quaternion's largest component is always at least 1/2, so the other three each fit in
// [-1/sqrt(2), 1/sqrt(2)] and the largest can be recomputed from the unit-norm constraint. A
// 2-bit index plus three 10-bit components spends exactly 32 bits.
inline constexpr int kQuatBits = 10;
inline constexpr int kQuatSelectorShift = 3 * kQuatBits;  // dropped index, at bits 30-31
inline constexpr uint32_t kQuatMask = (1u << kQuatBits) - 1;
inline constexpr int32_t kQuatMax = (1 << (kQuatBits - 1)) - 1;  // +/- 511, a symmetric range
inline constexpr double kQuatLimit = 0.70710678118654752440;     // 1/sqrt(2)
inline constexpr double kMinQuatNorm = 1.0e-9;

/**
 * @brief Packs any quaternion into 4 bytes, falling back to identity if it names no rotation.
 */
inline uint32_t encodeQuaternion(const geometry_msgs::msg::Quaternion& quat) {
  double q[4] = {quat.x, quat.y, quat.z, quat.w};
  double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  // NaN compares false against everything, so test for a usable norm rather than against a bad one
  if (!std::isfinite(norm) || norm < kMinQuatNorm) {
    q[0] = q[1] = q[2] = 0.0;
    q[3] = norm = 1.0;
  }
  for (double& component : q) component /= norm;

  int largest = 0;
  for (int i = 1; i < 4; ++i) {
    if (std::fabs(q[i]) > std::fabs(q[largest])) largest = i;
  }
  // Negating a quaternion leaves the rotation unchanged, so flipping the dropped component
  // positive means its sign never has to be stored
  const double sign = (q[largest] < 0.0) ? -1.0 : 1.0;

  uint32_t packed = static_cast<uint32_t>(largest) << kQuatSelectorShift;
  int shift = kQuatSelectorShift;
  for (int i = 0; i < 4; ++i) {
    if (i == largest) continue;
    shift -= kQuatBits;  // the kept components land at bits 20, 10, then 0
    const auto counts = static_cast<int32_t>(
        roundSaturating(q[i] * sign / kQuatLimit * kQuatMax, -kQuatMax, kQuatMax));
    packed |= (static_cast<uint32_t>(counts) & kQuatMask) << shift;
  }
  return packed;
}

/**
 * @brief Unpacks any 32-bit word into a unit quaternion.
 */
inline geometry_msgs::msg::Quaternion decodeQuaternion(uint32_t packed) {
  const uint32_t largest = packed >> kQuatSelectorShift;  // two bits wide, so always in [0, 3]

  double q[4];
  double sum_squares = 0.0;
  int shift = kQuatSelectorShift;
  for (uint32_t i = 0; i < 4; ++i) {
    if (i == largest) continue;
    shift -= kQuatBits;
    const uint32_t raw = (packed >> shift) & kQuatMask;
    const int32_t counts = (raw > static_cast<uint32_t>(kQuatMax))  // sign-extend the 10-bit field
                               ? static_cast<int32_t>(raw) - static_cast<int32_t>(kQuatMask + 1)
                               : static_cast<int32_t>(raw);
    q[i] = static_cast<double>(counts) / kQuatMax * kQuatLimit;
    sum_squares += q[i] * q[i];
  }
  q[largest] = std::sqrt(std::max(0.0, 1.0 - sum_squares));

  // Unit-norm by construction for a well-formed word, so this only rescales the corrupt case
  const double norm = std::sqrt(sum_squares + q[largest] * q[largest]);

  geometry_msgs::msg::Quaternion quat;
  quat.x = q[0] / norm;
  quat.y = q[1] / norm;
  quat.z = q[2] / norm;
  quat.w = q[3] / norm;
  return quat;
}

// --- Payload Cursors ---

/**
 * @class PayloadWriter
 * @brief Appends little-endian fields to a payload, tracking the offset.
 */
class PayloadWriter {
 public:
  explicit PayloadWriter(DatPayload& buf) : buf_(buf) {}

  template <typename T>
  void put(T value) {
    assert(off_ + sizeof(T) <= buf_.size());
    std::memcpy(buf_.data() + off_, &value, sizeof(T));
    off_ += sizeof(T);
  }

  uint8_t bytesWritten() const { return static_cast<uint8_t>(off_); }

 private:
  DatPayload& buf_;
  size_t off_ = 0;
};

/**
 * @class PayloadReader
 * @brief Consumes little-endian fields from a payload, tracking the offset.
 */
class PayloadReader {
 public:
  explicit PayloadReader(const DatPayload& buf) : buf_(buf) {}

  template <typename T>
  T get() {
    assert(off_ + sizeof(T) <= buf_.size());
    T value;
    std::memcpy(&value, buf_.data() + off_, sizeof(T));
    off_ += sizeof(T);
    return value;
  }

 private:
  const DatPayload& buf_;
  size_t off_ = 0;
};

}  // namespace detail

// --- Packet Codec ---

/**
 * @brief Encodes an AgentStatus into a Seatrac DAT payload.
 * @param status The status to encode.
 * @param buf The payload to populate.
 * @return The number of bytes written (kStatusPacketLen).
 */
inline uint8_t encodeStatus(const coug_interfaces::msg::AgentStatus& status, DatPayload& buf) {
  detail::PayloadWriter out(buf);  // mirrors decodeStatus(); the two are edited together
  out.put<uint8_t>(static_cast<uint8_t>(MsgId::RESP_STATUS));

  const auto& pose = status.local_odometry;
  out.put(detail::encodeMeters(pose.position.x));
  out.put(detail::encodeMeters(pose.position.y));
  out.put(detail::encodeMeters(pose.position.z));
  out.put(detail::encodeQuaternion(pose.orientation));

  for (int i = 0; i < kCovDim; ++i) {
    out.put(detail::encodeVariance(status.odometry_covariance[i * kCovStride]));
  }

  out.put(detail::encodeMeters(status.pressure_depth));
  out.put(detail::encodeQuaternion(status.imu_orientation));

  assert(out.bytesWritten() == kStatusPacketLen);
  return out.bytesWritten();
}

/**
 * @brief Decodes a Seatrac DAT payload into an AgentStatus.
 * @param buf The received payload.
 * @param len The payload length.
 * @param status The status to populate (header is left untouched).
 * @return True if the payload is long enough and carries the RESP_STATUS discriminator.
 */
inline bool decodeStatus(const DatPayload& buf, uint8_t len,
                         coug_interfaces::msg::AgentStatus& status) {
  if (len < kStatusPacketLen) return false;
  detail::PayloadReader in(buf);  // mirrors encodeStatus(); the two are edited together
  if (in.get<uint8_t>() != static_cast<uint8_t>(MsgId::RESP_STATUS)) return false;

  auto& pose = status.local_odometry;
  pose.position.x = detail::decodeMeters(in.get<int16_t>());
  pose.position.y = detail::decodeMeters(in.get<int16_t>());
  pose.position.z = detail::decodeMeters(in.get<int16_t>());
  pose.orientation = detail::decodeQuaternion(in.get<uint32_t>());

  status.odometry_covariance.fill(0.0);  // the off-diagonal entries are never transmitted
  for (int i = 0; i < kCovDim; ++i) {
    status.odometry_covariance[i * kCovStride] = detail::decodeVariance(in.get<uint16_t>());
  }

  status.pressure_depth = detail::decodeMeters(in.get<int16_t>());
  status.imu_orientation = detail::decodeQuaternion(in.get<uint32_t>());
  return true;
}

}  // namespace coug_comms::utils
