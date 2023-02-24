// Copyright 2023 Tier IV, Inc.
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
// limitations under the License

#ifndef VTL_ADAPTER__EVE_VTL_SPECIFICATION_HPP_
#define VTL_ADAPTER__EVE_VTL_SPECIFICATION_HPP_

#include <memory>
#include <string>

namespace eve_vtl_spec
{
static const std::string KEY_TYPE = "type";
static const std::string VALUE_TYPE = "eva_beacon_system";

static const std::string KEY_ID = "eva_beacon_system:id";
static constexpr uint8_t MAX_VALUE_ID = 254;
static constexpr uint8_t MIN_VALUE_ID = 1;

static const std::string KEY_SECTION = "eva_beacon_system:ref:section";
static const std::string VALUE_SECTION_REQ = "REQUESTING";
static const std::string VALUE_SECTION_NULL = "";

static const std::string KEY_PERMIT_STATE = "eva_beacon_system:ref:permit_state";
static const std::string VALUE_PERMIT_STATE_DRIVING = "DRIVING";
static const std::string VALUE_PERMIT_STATE_MANUAL = "MANUAL";
static const std::string VALUE_PERMIT_STATE_EMERGENCY = "EMERGENCY";
static const std::string VALUE_PERMIT_STATE_ARRIVAL_GOAL = "ARRIVAL_GOAL";
static const std::string VALUE_PERMIT_STATE_ENGAGE = "ENGAGE";
static const std::string VALUE_PERMIT_STATE_NULL = "";

static const std::string KEY_REQUEST_BIT = "eva_beacon_system:ref:request_bit";
static const std::string KEY_EXPECT_BIT = "eva_beacon_system:ref:expect_bit";
static constexpr uint8_t MAX_VALUE_BIT = 0x0f;
static constexpr uint8_t MIN_VALUE_BIT = 0x00;
enum class TD_BIT : uint8_t
{
  STRAIGHT = 0x01,
  RIGHT = 0x02,
  LEFT = 0x04,
  INVALID = MAX_VALUE_BIT
};

static const std::string KEY_RESPONSE_TYPE = "eva_beacon_system:ref:response_type";
static const std::string VALUE_RESPONSE_TYPE_ALWAYS = "ALWAYS";
static const std::string VALUE_RESPONSE_TYPE_AND = "AND";
static const std::string VALUE_RESPONSE_TYPE_MATCH = "MATCH";

static const std::string KEY_MODE = "eva_beacon_system:ref:mode";
static const std::string VALUE_MODE_FIXED = "FIXED_VALUE";
static const std::string VALUE_MODE_TURN_DIRECTION = "TURN_DIRECTION";

}  // namespace eve_vtl_spec

#endif  // VTL_ADAPTER__EVE_VTL_SPECIFICATION_HPP_