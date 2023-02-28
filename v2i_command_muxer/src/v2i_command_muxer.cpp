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

#include "v2i_command_muxer/v2i_command_muxer.hpp"

namespace v2i_command_muxer
{

V2iCommandMuxer::V2iCommandMuxer(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("v2i_command_muxer", options)
{
  using namespace std::placeholders;
  // Parameter settings
  std::vector<std::string> input_topics;
  std::string output_topic;
  double rate;
  
  declare_parameter("input_topics", std::vector<std::string>());
  input_topics = get_parameter("input_topics").as_string_array();
  declare_parameter("output_topic", std::string());
  output_topic = get_parameter("output_topic").as_string();
  rate = this->declare_parameter<double>("timer_rate", 10.0);

  // Subscription
  auto group = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = group;
  for (const auto& topic : input_topics) {
    std::function<void(const CommandArr::SharedPtr)> callback_func
      = std::bind(&V2iCommandMuxer::onCommand, this, _1, topic);
    cmdarr_sub_.emplace_back(this->create_subscription<CommandArr>(topic, 1,
      callback_func, subscriber_option));
  }

  // Publisher
  cmdarr_pub_ = this->create_publisher<CommandArr>(output_topic, rclcpp::QoS{1});

  // Timer
  publish_timer_ = create_timer(this, this->get_clock(),
    rclcpp::Rate(rate).period(), std::bind(&V2iCommandMuxer::onTimer, this),
    group);
}

void V2iCommandMuxer::onCommand(const CommandArr::SharedPtr input,
  const std::string topic_name)
{
  cmdarr_data_[topic_name] = input;
}

void V2iCommandMuxer::onTimer()
{
  const auto cmdarr_ptr = muxAllCommandArr();
  if (!cmdarr_ptr) {
    return;
  }
  cmdarr_pub_->publish(*cmdarr_ptr);
  deleteAllCommandArr();
}

CommandArr::SharedPtr V2iCommandMuxer::muxAllCommandArr() const
{
  CommandArr::SharedPtr arr_out(new CommandArr);
  auto& cmd_out = arr_out->commands;
  for (const auto& cmdmap : cmdarr_data_) {
    const auto& arr_in = cmdmap.second;
    if (!arr_in) {
      continue;
    }
    const auto& cmd_in = arr_in->commands;
    if (cmd_in.empty()) {
      continue;
    }
    cmd_out.insert(cmd_out.end(), cmd_in.begin(), cmd_in.end());
    if (rclcpp::Time(arr_out->stamp) < rclcpp::Time(arr_in->stamp)) {
      arr_out->stamp = arr_in->stamp;
    }
  }
  if (arr_out->commands.empty()) {
    arr_out.reset();
  }
  return arr_out;
}

void V2iCommandMuxer::deleteAllCommandArr()
{
  for (auto& cmdarr : cmdarr_data_) {
    cmdarr.second.reset();
  }
}

}  // namespace v2i_command_muxer

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(v2i_command_muxer::V2iCommandMuxer)