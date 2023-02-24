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

#ifndef VTL_ADAPTER__EVE_VTL_INTERFACE_CONVERTER_HPP_
#define VTL_ADAPTER__EVE_VTL_INTERFACE_CONVERTER_HPP_

#include <memory>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "tier4_v2x_msgs/msg/infrastructure_command.hpp"
#include "tier4_v2x_msgs/msg/key_value.hpp"
#include "autoware_state_machine_msgs/msg/state_machine.hpp"

#include "vtl_adapter/eve_vtl_attribute.hpp"

namespace eve_vtl_interface_converter
{

using EveVTLAttr = eve_vtl_attribute::EveVTLAttr;
using InfrastructureCommand = tier4_v2x_msgs::msg::InfrastructureCommand;
using StateMachine = autoware_state_machine_msgs::msg::StateMachine;

class EveVTLInterfaceConverter
{
public:
  explicit EveVTLInterfaceConverter(const InfrastructureCommand& input_command);

  const std::shared_ptr<EveVTLAttr>& vtlAttribute() const;
  const InfrastructureCommand& command() const;
  std::optional<uint8_t> request(
    const StateMachine::ConstSharedPtr& state) const;
  bool response(const uint8_t& response_bit) const;
private:
  bool init(const std::vector<tier4_v2x_msgs::msg::KeyValue>& custom_tags);
  std::string convertInfraCommand(const uint8_t& input_command) const;
  std::optional<std::string> convertADState(
    const StateMachine::ConstSharedPtr& state) const;

  InfrastructureCommand command_;
  std::shared_ptr<EveVTLAttr> vtl_attr_;
};

}  // namespace eve_vtl_interface_converter

#endif  // VTL_ADAPTER__EVE_VTL_INTERFACE_CONVERTER_HPP_