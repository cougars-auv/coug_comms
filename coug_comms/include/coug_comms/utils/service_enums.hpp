// Copyright 2026 BYU FROST Lab
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

enum class ServiceOutcome : std::uint8_t {
  kSucceeded = 0,
  kFailed,
  kQueued,
};

inline auto toString(ServiceOutcome outcome) -> std::string {
  switch (outcome) {
    case ServiceOutcome::kSucceeded:
      return "SUCCEEDED";
    case ServiceOutcome::kFailed:
      return "FAILED";
    case ServiceOutcome::kQueued:
      return "QUEUED";
  }
  return "UNKNOWN";
}

}  // namespace coug_comms::utils
