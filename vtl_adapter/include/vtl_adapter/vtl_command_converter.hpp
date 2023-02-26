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

#include "vtl_adapter/interface_converter_data_pipeline.hpp"
#include "vtl_adapter/eve_vtl_interface_converter.hpp"

namespace vtl_command_converter
{

using MainInputCommandArr = tier4_v2x_msgs::msg::InfrastructureCommandArray;
using MainOutputCommandArr = v2i_interface_msgs::msg::InfrastructureCommandArray;
using MainOutputCommand = v2i_interface_msgs::msg::InfrastructureCommand;
using InterfaceConverter = eve_vtl_interface_converter::EveVTLInterfaceConverter;

using SubInputState = autoware_state_machine_msgs::msg::StateMachine;

using InterfaceConverterMap =
  std::unordered_map<uint8_t, std::shared_ptr<InterfaceConverter>>;
using IFConverterDataPipeline =
  interface_converter_data_pipeline::IFConverterDataPipeline;

class VtlCommandConverter
{
public:
  VtlCommandConverter();
  void init(rclcpp::Node* node);
  std::shared_ptr<IFConverterDataPipeline> converterPipeline();
private:
  rclcpp::Node* node_;

  // Publisher
  rclcpp::Publisher<MainOutputCommandArr>::SharedPtr command_pub_;

  // Subscription
  rclcpp::Subscription<MainInputCommandArr>::SharedPtr command_sub_;
  rclcpp::Subscription<SubInputState>::SharedPtr state_sub_;

  // Callback
  void onCommand(const MainInputCommandArr::ConstSharedPtr msg);
  void onState(const SubInputState::ConstSharedPtr msg);

  // Preprocess
  std::shared_ptr<InterfaceConverterMap> createConverter(
    const MainInputCommandArr::ConstSharedPtr& original_command) const;
  std::optional<MainOutputCommandArr> requestCommand(
    const std::shared_ptr<InterfaceConverterMap>& converter_array) const;

  //member variables
  SubInputState::ConstSharedPtr state_;
  std::shared_ptr<IFConverterDataPipeline> converter_pipeline_;
};

}  // namespace vtl_command_converter

#endif  // VTL_ADAPTER__VTL_COMMAND_CONVERTER_HPP_
