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

#ifndef VTL_ADAPTER__VTL_STATE_CONVERTER_HPP_
#define VTL_ADAPTER__VTL_STATE_CONVERTER_HPP_

#include "rclcpp/rclcpp.hpp"

// input and output
#include "tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp"
#include "v2i_interface_msgs/msg/infrastructure_state_array.hpp"

#include "vtl_adapter/interface_converter_data_pipeline.hpp"

namespace vtl_state_converter
{

using InputStateArr = v2i_interface_msgs::msg::InfrastructureStateArray;
using OutputStateArr = tier4_v2x_msgs::msg::VirtualTrafficLightStateArray;
using OutputState = tier4_v2x_msgs::msg::VirtualTrafficLightState;
using InterfaceConverter = eve_vtl_interface_converter::EveVTLInterfaceConverter;
using InterfaceConverterMap =
  std::unordered_map<uint8_t, std::shared_ptr<InterfaceConverter>>;
using IFConverterDataPipeline =
  interface_converter_data_pipeline::IFConverterDataPipeline;

class VtlStateConverter
{
public:
  void init(rclcpp::Node* node);
  bool acceptConverterPipeline(
    std::shared_ptr<IFConverterDataPipeline> converter_pipeline);
private:
  // Publisher
  rclcpp::Publisher<OutputStateArr>::SharedPtr state_pub_;

  // Subscription
  rclcpp::Subscription<InputStateArr>::SharedPtr state_sub_;

  void onState(const InputStateArr::ConstSharedPtr msg);

  std::optional<OutputStateArr> createState(
    const InputStateArr::ConstSharedPtr& msg);
  std::shared_ptr<IFConverterDataPipeline> converter_pipeline_;
};

}  // namespace vtl_state_converter

#endif  // VTL_ADAPTER__VTL_STATE_CONVERTER_HPP_