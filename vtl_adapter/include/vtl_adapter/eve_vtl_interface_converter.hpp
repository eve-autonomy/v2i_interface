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

#ifndef VTL_ADAPTER__EVE_VTL_INTERFACE_CONVERTER_HPP_
#define VTL_ADAPTER__EVE_VTL_INTERFACE_CONVERTER_HPP_

#include <memory>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "tier4_v2x_msgs/msg/infrastructure_command.hpp"
#include "tier4_v2x_msgs/msg/key_value.hpp"

#include "autoware_adapi_v1_msgs/msg/route_state.hpp"
#include "autoware_adapi_v1_msgs/msg/route.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "eve_cmd_gate_msgs/msg/engage_request_state.hpp"

#include "vtl_adapter/eve_vtl_attribute.hpp"

namespace eve_vtl_interface_converter
{

using EveVTLAttr = eve_vtl_attribute::EveVTLAttr;
using InfrastructureCommand = tier4_v2x_msgs::msg::InfrastructureCommand;

using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
using Route = autoware_adapi_v1_msgs::msg::Route;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
using RouteData = autoware_adapi_v1_msgs::msg::RouteData;
using AutonomousDrivingStartButton =eve_cmd_gate_msgs::msg::EngageRequestState;

class EveVTLInterfaceConverter
{
public:
  EveVTLInterfaceConverter(
    const InfrastructureCommand& input_command, rclcpp::Node* node);

  const std::shared_ptr<EveVTLAttr>& vtlAttribute() const;
  const InfrastructureCommand& command() const;
  std::optional<uint8_t> request() const;
  bool response(const uint8_t& response_bit) const;
private:
  bool init(const InfrastructureCommand& input_command);
  std::string convertInfraCommand(const uint8_t& input_command) const;
  std::optional<std::string> convertADState() const;

    // Subscription
  rclcpp::Subscription<RouteState>::SharedPtr sub_routing_state_;
  rclcpp::Subscription<Route>::SharedPtr sub_routing_route_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_state_;
  rclcpp::Subscription<AutonomousDrivingStartButton>::SharedPtr sub_autonomous_driving_start_button_;

  // Callback
  void onState(const RouteState::ConstSharedPtr msg);
  void onRoute(const Route::ConstSharedPtr msg);
  void onOperationModeState(const OperationModeState::ConstSharedPtr msg);
  void onAutonomousDrivingStartButton(const AutonomousDrivingStartButton::ConstSharedPtr msg);

  InfrastructureCommand command_;
  std::shared_ptr<EveVTLAttr> vtl_attr_;
  rclcpp::Node* node_;
  uint16_t state_;
  Route route_;
  bool is_autoware_control_enabled_;
  bool is_in_transition_;
  uint8_t mode_;
  bool is_accept_;
  bool is_request_;
};

}  // namespace eve_vtl_interface_converter

#endif  // VTL_ADAPTER__EVE_VTL_INTERFACE_CONVERTER_HPP_
