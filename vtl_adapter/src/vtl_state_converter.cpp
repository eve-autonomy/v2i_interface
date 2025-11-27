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

#include "vtl_adapter/vtl_state_converter.hpp"
#include "vtl_adapter/interface_converter_data_pipeline.hpp"

namespace vtl_state_converter
{

constexpr static unsigned int ERROR_THROTTLE_MSEC = 1000;

void VtlStateConverter::init(rclcpp::Node* node)
{
  if (!node) {
    return;
  }
  using namespace std::placeholders;

  node_ = node;
  auto group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = group;

  // Subscription
  state_sub_ = node->create_subscription<InputStateArr>(
    "~/input/infrastructure_states", 1,
    std::bind(&VtlStateConverter::onState, this, _1),
    subscriber_option);
  // Publisher
  state_pub_ = node->create_publisher<OutputStateArr>(
    "~/output/infrastructure_states",
    rclcpp::QoS{1});
  RCLCPP_INFO(node_->get_logger(),
    "VtlStateConverter: initialized.");
}

bool VtlStateConverter::acceptConverterPipeline(
  std::shared_ptr<IFConverterDataPipeline> converter_pipeline)
{
  if (converter_pipeline_) {
    return false;
  }
  converter_pipeline_ = converter_pipeline;
  RCLCPP_INFO(
    node_->get_logger(),
    "VtlStateConverter: converter pipeline is accepted.");
  return true;
}

void VtlStateConverter::onState(const InputStateArr::ConstSharedPtr msg)
{
  state_ = msg;
}

void VtlStateConverter::onCommand(const MainInputCommandArr::ConstSharedPtr msg)
{
  if (msg->commands.empty()){
    return;
  }
  if (!converter_pipeline_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "VtlStateConverter:%s: converter pipeline is not set.", __func__);
    return;
  }
  else if (!converter_pipeline_->load()) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "VtlStateConverter:%s: converter pipeline is not loaded.", __func__);
    return;
  }
  auto isNotFinalized = [](const MainInputCommand& cmd) {
    return (cmd.state != MainInputCommand::FINALIZED);
  };

  std::map<std::string, OutputState> merged_state_map; 
  
  const auto& cmd_arr = msg->commands;
  const auto is_all_commands_finalized =
    (std::count_if(cmd_arr.begin(), cmd_arr.end(), isNotFinalized) == 0);

  std::optional<OutputStateArr> output_state = OutputStateArr();
  output_state->stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  if(!is_all_commands_finalized){
    const auto received_state = createState();
    if (!received_state) {
      RCLCPP_DEBUG(node_->get_logger(),
      "VtlStateConverter:%s: no valid state is found.", __func__);
    }else{
      merged_state_map.insert(received_state.value().begin(),received_state.value().end());
    }

    const auto self_approval_state = createSelfApprovalState();
    if (!self_approval_state) {
      RCLCPP_DEBUG(node_->get_logger(),
      "VtlStateConverter:%s: no valid self approve state is found.", __func__);
    }else{
      merged_state_map.insert(self_approval_state.value().begin(),self_approval_state.value().end());
    }
    
    if(merged_state_map.size() == 0){
      output_state = std::nullopt;
    }
    for (const auto& [key, value] : merged_state_map){
      output_state->states.emplace_back(value);
    }
  }
  if(output_state.has_value()){
      state_pub_->publish(output_state.value());
  }
}

std::optional<std::map<std::string, OutputState>>
  VtlStateConverter::createState()
{
  const auto converter_multimap = converter_pipeline_->load();
  std::map<std::string, OutputState> output_state_arr_map;

  for (const auto& state : state_->states) {
    if (converter_multimap->count(state.id) < 1) {
      RCLCPP_DEBUG(node_->get_logger(), 
        "VtlStateConverter:%s: no converter is found for id:%d.",
        __func__, state.id);
      continue;
    }
    auto range = converter_multimap->equal_range(state.id);
    for (auto it = range.first; it != range.second; ++it) {
      const auto& converter = it->second;
      const auto& attr = converter->vtlAttribute();
      if (!attr) {
        RCLCPP_DEBUG(node_->get_logger(),
          "VtlStateConverter:%s: no attribute is found for id:%d.",
          __func__, state.id);
        continue;
      }
      else if (!attr->isValidAttr()) {
        RCLCPP_DEBUG(node_->get_logger(),
          "VtlStateConverter:%s: invalid attribute is found for id:%d.",
          __func__, state.id);
        continue;
      }
      OutputState output_state;
      output_state.stamp = state_->stamp;
      output_state.type = attr->type();
      output_state.id = converter->command().id;
      output_state.approval = converter->response(state.state);
      output_state.is_finalized = true;
      output_state_arr_map.insert(std::make_pair(output_state.id,output_state));
    }
  }
  if (output_state_arr_map.empty()) {
    RCLCPP_DEBUG(node_->get_logger(),
      "VtlStateConverter:%s: no valid state is found.", __func__);
    return std::nullopt;
  }
  return output_state_arr_map;
}

std::optional<std::map<std::string, OutputState>>
 VtlStateConverter::createSelfApprovalState()
{
  const auto converter_map = converter_pipeline_->load();
  if (!converter_map) {
    RCLCPP_DEBUG(node_->get_logger(),
      "SelfApprovalTimer:%s: converter map is not loaded.", __func__);
    return std::nullopt;
  }
  else if (converter_map->empty()) {
    RCLCPP_DEBUG(node_->get_logger(),
      "SelfApprovalTimer:%s: converter map is empty.", __func__);
    return std::nullopt;
  }

  std::map<std::string, OutputState> output_state_arr_map;
  std::unordered_set<uint8_t> id_set;
  const auto stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  for (const auto& elem : *converter_map) {
    const auto& converter = elem.second;
    const auto& attr = converter->vtlAttribute();
    if (!attr) {
      RCLCPP_DEBUG(node_->get_logger(),
        "SelfApprovalTimer:%s: vtl attribute is not set at id=%d.",
        __func__, elem.first);
      continue;
    }
    else if (!attr->isValidAttr()) {
      RCLCPP_DEBUG(node_->get_logger(),
        "SelfApprovalTimer:%s: vtl attribute is not valid at id=%d.",
        __func__, elem.first);
      continue;
    }
    else if (!attr->isSelfApproval()) {
      RCLCPP_DEBUG(node_->get_logger(),
        "SelfApprovalTimer:%s: vtl attribute is not self approval at id=%d.",
        __func__, elem.first);
      continue;
    }

    const uint8_t id = attr->id().value();
    if (id_set.find(id) != id_set.end()) {
      continue;
    }
    id_set.emplace(id);
    OutputState output_state;
    output_state.stamp = stamp;
    output_state.type = attr->type();
    output_state.id = converter->command().id;
    output_state.approval = converter->response(attr->expectBit().value());
    output_state.is_finalized = true;
    output_state_arr_map.insert(std::make_pair(output_state.id,output_state));
  }
  if (output_state_arr_map.empty()) {
    RCLCPP_DEBUG(node_->get_logger(),
      "SelfApprovalTimer:%s: self input state is empty.", __func__);
    return std::nullopt;
  }
  return output_state_arr_map;
}
}  // namespace vtl_state_converter
