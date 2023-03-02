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

#ifndef VTL_ADAPTER__VTL_ADAPTER_HPP_
#define VTL_ADAPTER__VTL_ADAPTER_HPP_

#include "vtl_adapter/vtl_command_converter.hpp"
#include "vtl_adapter/vtl_state_converter.hpp"
#include "vtl_adapter/self_approval_timer.hpp"

namespace vtl_adapter
{
using VtlCommandConverter = vtl_command_converter::VtlCommandConverter;
using VtlStateConverter = vtl_state_converter::VtlStateConverter;
using SelfApprovalTimer = self_approval_timer::SelfApprovalTimer;

class VtlAdapterNode : public rclcpp::Node
{
public:
  explicit VtlAdapterNode(const rclcpp::NodeOptions & options);
private:
  // command converter
  VtlCommandConverter command_converter_;
  
  // state converter
  VtlStateConverter state_converter_;

  // self approval timer
  SelfApprovalTimer self_approval_timer_;
};

}  // namespace vtl_adapter

#endif  // VTL_ADAPTER__VTL_ADAPTER_HPP_
