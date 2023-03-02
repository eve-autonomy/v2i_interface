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

#ifndef VTL_ADAPTER__SELF_APPROVAL_TIMER_HPP_
#define VTL_ADAPTER__SELF_APPROVAL_TIMER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

// input and output
#include "v2i_interface_msgs/msg/infrastructure_state_array.hpp"

#include "vtl_adapter/interface_converter_data_pipeline.hpp"

namespace self_approval_timer
{

using InputStateArr = v2i_interface_msgs::msg::InfrastructureStateArray;
using InputState = v2i_interface_msgs::msg::InfrastructureState;
using IFConverterDataPipeline =
  interface_converter_data_pipeline::IFConverterDataPipeline;

class SelfApprovalTimer
{
public:
  void init(rclcpp::Node* node);
  bool acceptConverterPipeline(
    std::shared_ptr<IFConverterDataPipeline> converter_pipeline);
private:
  // Functions
  void onTimer();
  std::optional<InputStateArr> createState();

  // Publisher
  rclcpp::Publisher<InputStateArr>::SharedPtr state_pub_;

  // Publish timer
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Data pipeline
  std::shared_ptr<IFConverterDataPipeline> converter_pipeline_;

  rclcpp::Node* node_;
};

}  // namespace self_approval_timer

#endif  // VTL_ADAPTER__SELF_APPROVAL_TIMER_HPP_
