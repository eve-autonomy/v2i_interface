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

#ifndef V2I_COMMAND_MUXER__V2I_COMMAND_MUXER_HPP_
#define V2I_COMMAND_MUXER__V2I_COMMAND_MUXER_HPP_

#include <optional>
#include "rclcpp/rclcpp.hpp"

// input and output
#include "v2i_interface_msgs/msg/infrastructure_command_array.hpp"

namespace v2i_command_muxer
{

using CommandArr = v2i_interface_msgs::msg::InfrastructureCommandArray;
using Command = v2i_interface_msgs::msg::InfrastructureCommand;

class V2iCommandMuxer : public rclcpp::Node
{
public:
  explicit V2iCommandMuxer(const rclcpp::NodeOptions & options);
private:
  // Functions
  void onCommand(const CommandArr::SharedPtr input,
    const std::string topic_name);
  void onTimer();

  CommandArr::SharedPtr muxAllCommandArr() const;
  void deleteAllCommandArr();

  // Publisher
  rclcpp::Publisher<CommandArr>::SharedPtr cmdarr_pub_;

  // Subscription
  std::vector<rclcpp::Subscription<CommandArr>::SharedPtr> cmdarr_sub_;

  // Data storage
  std::unordered_map<std::string, CommandArr::SharedPtr> cmdarr_data_;

  // Publish timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace v2i_command_muxer

#endif  // V2I_COMMAND_MUXER__V2I_COMMAND_MUXER_HPP_
