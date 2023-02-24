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

#ifndef VTL_ADAPTER__VTL_COMMAND_CONVERTER_HPP_
#define VTL_ADAPTER__VTL_COMMAND_CONVERTER_HPP_

#include <optional>
#include "rclcpp/rclcpp.hpp"

// main input and output
#include "tier4_v2x_msgs/msg/infrastructure_command_array.hpp"
#include "v2i_interface_msgs/msg/infrastructure_command_array.hpp"

// sub input
#include "autoware_state_machine_msgs/msg/state_machine.hpp"

#include "vtl_adapter/eve_vtl_interface_converter.hpp"

namespace vtl_command_converter
{

using MainInputCommandArr = tier4_v2x_msgs::msg::InfrastructureCommandArray;
using MainOutputCommandArr = v2i_interface_msgs::msg::InfrastructureCommandArray;
using MainOutputCommand = v2i_interface_msgs::msg::InfrastructureCommand;
using InterfaceConverter = eve_vtl_interface_converter::EveVTLInterfaceConverter;

using SubInputState = autoware_state_machine_msgs::msg::StateMachine;

using InterfaceConverterArr = std::vector<std::shared_ptr<InterfaceConverter>>;

class VtlCommandConverterNode : public rclcpp::Node
{
public:
  explicit VtlCommandConverterNode(const rclcpp::NodeOptions & options);
private:
  // Publisher
  rclcpp::Publisher<MainOutputCommandArr>::SharedPtr command_pub_;

  // Subscription
  rclcpp::Subscription<MainInputCommandArr::ConstSharedPtr>::SharedPtr command_sub_;
  rclcpp::Subscription<SubInputState::ConstSharedPtr>::SharedPtr state_sub_;

  // Callback
  void onCommand(const MainInputCommandArr::ConstSharedPtr& msg);
  void onState(const SubInputState::ConstSharedPtr& msg);

  // Preprocess
  std::shared_ptr<InterfaceConverterArr> createConverter(
    const MainInputCommandArr::ConstSharedPtr& original_command) const;
  std::optional<MainOutputCommandArr> requestCommand(
    const std::shared_ptr<InterfaceConverterArr>& converter_array) const;

  //member variables
  SubInputState::ConstSharedPtr state_;
};

}  // namespace vtl_command_converter

#endif  // VTL_ADAPTER__VTL_COMMAND_CONVERTER_HPP_
