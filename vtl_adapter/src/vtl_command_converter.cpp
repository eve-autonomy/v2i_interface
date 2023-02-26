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

#include "vtl_adapter/vtl_command_converter.hpp"
#include "vtl_adapter/eve_vtl_interface_converter.hpp"

namespace vtl_command_converter
{

VtlCommandConverter::VtlCommandConverter(rclcpp::Node* node)
: node_(node)
{
  using namespace std::placeholders;

  auto group = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = group;

  // Subscription
  command_sub_ = node_->create_subscription<MainInputCommandArr>(
    "/awapi/tmp/infrastructure_commands", 1,
    std::bind(&VtlCommandConverter::onCommand, this, _1),
    subscriber_option);
  state_sub_ = node_->create_subscription<SubInputState>(
    "/autoware_state_machine/state", 1,
    std::bind(&VtlCommandConverter::onState, this, _1),
    subscriber_option);
  // Publisher
  command_pub_ = node_->create_publisher<MainOutputCommandArr>(
    "/v2x/infrastructure_commands",
    rclcpp::QoS{1});
}

void VtlCommandConverter::onCommand(const MainInputCommandArr::ConstSharedPtr msg)
{
  const auto output_command = requestCommand(createConverter(msg));
  if (!output_command) {
    return;
  }
  command_pub_->publish(output_command.value());
}

void VtlCommandConverter::onState(const SubInputState::ConstSharedPtr msg)
{
  state_ = msg;
}

std::shared_ptr<InterfaceConverterArr> VtlCommandConverter::createConverter(
    const MainInputCommandArr::ConstSharedPtr& original_command) const
{
  std::shared_ptr<InterfaceConverterArr> converter_array(new InterfaceConverterArr());
  for (const auto& orig_elem : original_command->commands) {
    converter_array->emplace_back(new InterfaceConverter(orig_elem));
  }
  return converter_array;
}

std::optional<MainOutputCommandArr> VtlCommandConverter::requestCommand(
  const std::shared_ptr<InterfaceConverterArr>& converter_array) const
{
  MainOutputCommandArr command_array;
  for (const auto& elem : *converter_array) {
    const auto& attr = elem->vtlAttribute();
    const auto& req = elem->request(state_);
    if (!attr) {
      continue;
    }
    else if (!attr->id()) {
      continue;
    }
    if (!req) {
      continue;
    }
    MainOutputCommand command;
    {
      command.stamp = elem->command().stamp;
      command.id = attr->id().value();
      command.state = req.value();
    }
    command_array.commands.emplace_back(command);
  }
  if (command_array.commands.empty()) {
    return std::nullopt;
  }
  return command_array;
}

}  // namespace vtl_command_converter
