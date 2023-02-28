// Copyright 2023 TIER IV, Inc.
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

#ifndef VTL_ADAPTER__AUTOWARE_LANELET_SPECIFICATION_HPP_
#define VTL_ADAPTER__AUTOWARE_LANELET_SPECIFICATION_HPP_

#include <memory>
#include <string>

namespace aw_lanelet_spec
{

static const std::string KEY_TURN_DIRECTION = "turn_direction";
static const std::string VALUE_TURN_DIRECTION_STRAIGHT = "straight";
static const std::string VALUE_TURN_DIRECTION_RIGHT = "right";
static const std::string VALUE_TURN_DIRECTION_LEFT = "left";
static const std::string INVALID_VALUE_TURN_DIRECTION = "";

}  // namespace aw_lanelet_spec

#endif  // VTL_ADAPTER__AUTOWARE_LANELET_SPECIFICATION_HPP_