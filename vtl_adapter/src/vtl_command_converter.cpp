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

#include "vtl_adapter/vtl_command_converter.hpp"
#include "vtl_adapter/interface_converter_data_pipeline.hpp"
#include "vtl_adapter/eve_vtl_interface_converter.hpp"

namespace vtl_command_converter
{

VtlCommandConverter::VtlCommandConverter()
: converter_pipeline_(new IFConverterDataPipeline)
{}

void VtlCommandConverter::init(rclcpp::Node* node)
{
  if (!node) {
    return;
  }
  using namespace std::placeholders;

  auto group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = group;

  // Subscription
  command_sub_ = node->create_subscription<MainInputCommandArr>(
    "~/input/infrastructure_commands", 1,
    std::bind(&VtlCommandConverter::onCommand, this, _1),
    subscriber_option);
  state_sub_ = node->create_subscription<SubInputState>(
    "/autoware_state_machine/state", 1,
    std::bind(&VtlCommandConverter::onState, this, _1),
    subscriber_option);
  // Publisher
  command_pub_ = node->create_publisher<MainOutputCommandArr>(
    "~/output/infrastructure_commands",
    rclcpp::QoS{1});
}


std::shared_ptr<IFConverterDataPipeline> VtlCommandConverter::converterPipeline()
{
  return converter_pipeline_;
}

void VtlCommandConverter::onCommand(const MainInputCommandArr::ConstSharedPtr msg)
{
  const auto converter_multimap = createConverter(msg);
  const auto output_command = requestCommand(converter_multimap);
  if (!output_command) {
    return;
  }
  command_pub_->publish(output_command.value());
  converter_pipeline_->add(converter_multimap);
}

void VtlCommandConverter::onState(const SubInputState::ConstSharedPtr msg)
{
  state_ = msg;
}

std::shared_ptr<InterfaceConverterMultiMap> VtlCommandConverter::createConverter(
    const MainInputCommandArr::ConstSharedPtr& original_command) const
{
  std::shared_ptr<InterfaceConverterMultiMap>
    converter_multimap(new InterfaceConverterMultiMap());
  for (const auto& orig_elem : original_command->commands) {
    if (orig_elem.state == MainInputCommand::NONE) {
      continue;
    }
    const auto converter(new InterfaceConverter(orig_elem));
    if (!converter->vtlAttribute()) {
      continue;
    }
    const auto id_opt = converter->vtlAttribute()->id();
    if (!id_opt) {
      continue;
    }
    converter_multimap->emplace(id_opt.value(), converter);
  }
  return converter_multimap;
}

std::optional<MainOutputCommandArr> VtlCommandConverter::requestCommand(
  const std::shared_ptr<InterfaceConverterMultiMap>& converter_multimap) const
{
  if (!converter_multimap) {
    return std::nullopt;
  }
  std::unordered_map<uint8_t, MainOutputCommand> command_map;
  for (const auto& [id, converter] : *converter_multimap) {
    const auto& req = converter->request(state_);
    if (!req) {
      continue;
    }
    if (command_map.find(id) != command_map.end()) {
      // If the same ID is found, the value is calculated by OR.
      // This is because the value is applied to multiple gpio signals.
      command_map.at(id).state |= req.value();
      continue;
    }
    MainOutputCommand command;
    {
      command.stamp = converter->command().stamp;
      command.id = id;
      command.state = req.value();
    }
    command_map.emplace(id, command);
  }

  MainOutputCommandArr command_array;
  for (const auto& [id, command] : command_map) {
    command_array.commands.emplace_back(command);
  }
  if (command_array.commands.empty()) {
    return std::nullopt;
  }
  return command_array;
}

}  // namespace vtl_command_converter
