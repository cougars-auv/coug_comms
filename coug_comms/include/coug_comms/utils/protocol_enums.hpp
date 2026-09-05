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

#include <cstdint>
#include <string>

namespace coug_comms::utils {

enum class MsgId : uint8_t {
  kServiceStart = 0x10,
  kServiceStop = 0x11,
  kServiceSurface = 0x12,
  kServiceHome = 0x13,
  kEmergencyStop = 0x14,
  kEmergencySurface = 0x15,
  kStatusRequest = 0x30,
  kStatusResponse = 0x31,
};

inline auto toString(MsgId msg) -> std::string {
  switch (msg) {
    case MsgId::kServiceStart:
      return "SRV_START";
    case MsgId::kServiceStop:
      return "SRV_STOP";
    case MsgId::kServiceSurface:
      return "SRV_SURFACE";
    case MsgId::kServiceHome:
      return "SRV_HOME";
    case MsgId::kEmergencyStop:
      return "SRV_EMERGENCY_STOP";
    case MsgId::kEmergencySurface:
      return "SRV_EMERGENCY_SURFACE";
    case MsgId::kStatusRequest:
      return "REQ_STATUS";
    case MsgId::kStatusResponse:
      return "RESP_STATUS";
  }
  return "UNKNOWN";
}

}  // namespace coug_comms::utils
