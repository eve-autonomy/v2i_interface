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

#include <unordered_set>
#include "vtl_adapter/self_approval_timer.hpp"
#include "vtl_adapter/interface_converter_data_pipeline.hpp"

namespace self_approval_timer
{

constexpr static unsigned int ERROR_THROTTLE_MSEC = 1000;

void SelfApprovalTimer::init(rclcpp::Node* node)
{
  if (!node) {
    return;
  }
  node_ = node;
  double rate;
  rate = node->declare_parameter<double>("timer_rate", 10.0);

  using namespace std::placeholders;
  group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = group_;

  // Timer
  publish_timer_ = create_timer(node, node->get_clock(),
    rclcpp::Rate(rate).period(), std::bind(&SelfApprovalTimer::onTimer, this),
    group_);
  
  // Publisher
  state_pub_ = node->create_publisher<InputStateArr>(
    "~/input/infrastructure_states",
    rclcpp::QoS{1});

  RCLCPP_INFO(node_->get_logger(),
    "SelfApprovalTimer: initialized.");
}

bool SelfApprovalTimer::acceptConverterPipeline(
  std::shared_ptr<IFConverterDataPipeline> converter_pipeline)
{
  if (converter_pipeline_) {
    return false;
  }
  converter_pipeline_ = converter_pipeline;
  RCLCPP_INFO(
    node_->get_logger(),
    "SelfApprovalTimer: converter pipeline is accepted.");
  return true;
}

void SelfApprovalTimer::onTimer()
{
  if (!converter_pipeline_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "SelfApprovalTimer:%s: converter pipeline is not set.", __func__);
    return;
  }
  else if (!converter_pipeline_->load()) {
    RCLCPP_DEBUG(node_->get_logger(),
      "SelfApprovalTimer:%s: converter pipeline is not loaded.", __func__);
    return;
  }
  const auto self_input_state = createState();
  if (!self_input_state) {
    RCLCPP_DEBUG(node_->get_logger(),
      "SelfApprovalTimer:%s: self input state is not created.", __func__);
    return;
  }
  state_pub_->publish(self_input_state.value());
}

std::optional<InputStateArr> SelfApprovalTimer::createState()
{
  const auto converter_map = converter_pipeline_->load();
  if (!converter_map) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "SelfApprovalTimer:%s: converter map is not loaded.", __func__);
    return std::nullopt;
  }
  else if (converter_map->empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "SelfApprovalTimer:%s: converter map is empty.", __func__);
    return std::nullopt;
  }

  InputStateArr self_input_state_arr;
  self_input_state_arr.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  std::unordered_set<uint8_t> id_set;
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
    InputState self_input_state;
    self_input_state.stamp = self_input_state_arr.stamp;
    self_input_state.id = id;
    self_input_state.state = attr->expectBit().value();
    self_input_state_arr.states.emplace_back(self_input_state);
  }
  if (self_input_state_arr.states.empty()) {
    RCLCPP_DEBUG(node_->get_logger(),
      "SelfApprovalTimer:%s: self input state is empty.", __func__);
    return std::nullopt;
  }
  return self_input_state_arr;
}
}  // namespace self_approval_timer
