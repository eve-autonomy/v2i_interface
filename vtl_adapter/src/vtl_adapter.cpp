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

#include "vtl_adapter/vtl_adapter.hpp"

namespace vtl_adapter
{

VtlAdapterNode::VtlAdapterNode(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("vtl_adapter", options)
{
  state_converter_.init(static_cast<rclcpp::Node*>(this));
  command_converter_.init(static_cast<rclcpp::Node*>(this));
  self_approval_timer_.init(static_cast<rclcpp::Node*>(this));
  state_converter_.acceptConverterPipeline(
    command_converter_.converterPipeline());
  self_approval_timer_.acceptConverterPipeline(
    command_converter_.converterPipeline());
}

}  // namespace vtl_adapter

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(vtl_adapter::VtlAdapterNode)
