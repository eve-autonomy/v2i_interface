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
  const auto output_state = createState(msg);
  if (!output_state) {
    RCLCPP_DEBUG(node_->get_logger(),
      "VtlStateConverter:%s: no valid state is found.", __func__);
    return;
  }
  state_pub_->publish(output_state.value());
}

std::optional<OutputStateArr>
  VtlStateConverter::createState(const InputStateArr::ConstSharedPtr& msg)
{
  const auto converter_multimap = converter_pipeline_->load();
  OutputStateArr output_state_arr;
  output_state_arr.stamp = msg->stamp;
  for (const auto& state : msg->states) {
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
      output_state.stamp = msg->stamp;
      output_state.type = attr->type();
      output_state.id = converter->command().id;
      output_state.approval = converter->response(state.state);
      output_state.is_finalized = true;
      output_state_arr.states.emplace_back(output_state);
    }
  }
  if (output_state_arr.states.empty()) {
    RCLCPP_DEBUG(node_->get_logger(),
      "VtlStateConverter:%s: no valid state is found.", __func__);
    return std::nullopt;
  }
  return output_state_arr;
}

}  // namespace vtl_state_converter
