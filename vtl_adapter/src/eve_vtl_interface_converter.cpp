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


#include "vtl_adapter/eve_vtl_interface_converter.hpp"
#include "vtl_adapter/eve_vtl_specification.hpp"
#include "vtl_adapter/autoware_lanelet_specification.hpp"

namespace eve_vtl_interface_converter
{

/*
***************************************************************
Class public function
***************************************************************
*/

EveVTLInterfaceConverter::EveVTLInterfaceConverter(const InfrastructureCommand& input_command)
  : command_(input_command)
{
  init(input_command);
}

const std::shared_ptr<EveVTLAttr>& EveVTLInterfaceConverter::vtlAttribute() const
{
  return vtl_attr_;
}

const InfrastructureCommand& EveVTLInterfaceConverter::command() const
{
  return command_;
}

std::optional<uint8_t> EveVTLInterfaceConverter::request(
    const StateMachine::ConstSharedPtr& state) const
{
  if (!vtl_attr_) {
    return std::nullopt;
  }
  const auto command_str = convertInfraCommand(command_.state);
  const auto state_str = convertADState(state);
  return vtl_attr_->request(command_str, state_str);
}

bool EveVTLInterfaceConverter::response(const uint8_t& response_bit) const
{
  if (!vtl_attr_) {
    return false;
  }
  return vtl_attr_->response(response_bit);
}

/*
***************************************************************
Class private function
***************************************************************
*/

bool EveVTLInterfaceConverter::init(const InfrastructureCommand& input_command)
{
  const auto type = input_command.type;
  // type == eva_bacon_system以外のときは初期化失敗
  if (type != eve_vtl_spec::VALUE_TYPE) {
    return false;
  }

  const auto& custom_tags = input_command.custom_tags;
  // custom_tagsが設定されてなければ初期化不要（失敗）
  if (custom_tags.empty()) {
    return false;
  }

  // custom_tagsを検索しやすい形に変換
  std::unordered_map<std::string, std::string> tags;
  for (const auto& tag : custom_tags) {
    tags[tag.key] = tag.value;
  }

  // id, modeが適切に設定されてなければ初期化失敗
  // 成功時はattribute変数にidとmodeを代入する
  std::shared_ptr<EveVTLAttr> attr(new EveVTLAttr);
  attr->setType(type);
  if (tags.find(aw_lanelet_spec::KEY_TURN_DIRECTION) != tags.end()) {
    attr->setTurnDirection(tags.at(aw_lanelet_spec::KEY_TURN_DIRECTION));
  }
  if (tags.find(eve_vtl_spec::KEY_MODE) != tags.end()) {
    attr->setMode(tags.at(eve_vtl_spec::KEY_MODE));
  }
  if (tags.find(eve_vtl_spec::KEY_ID) != tags.end()) {
    attr->setID(tags.at(eve_vtl_spec::KEY_ID));
  }
  if (tags.find(eve_vtl_spec::KEY_RESPONSE_TYPE) != tags.end()) {
    attr->setResponseType(tags.at(eve_vtl_spec::KEY_RESPONSE_TYPE));
  }
  if (tags.find(eve_vtl_spec::KEY_REQUEST_BIT) != tags.end()) {
    attr->setRequestBit(tags.at(eve_vtl_spec::KEY_REQUEST_BIT));
  }
  if (tags.find(eve_vtl_spec::KEY_EXPECT_BIT) != tags.end()) {
    attr->setExpectBit(tags.at(eve_vtl_spec::KEY_EXPECT_BIT));
  }
  if (tags.find(eve_vtl_spec::KEY_PERMIT_STATE) != tags.end()) {
    attr->setPermitState(tags.at(eve_vtl_spec::KEY_PERMIT_STATE));
  }
  if (tags.find(eve_vtl_spec::KEY_SECTION) != tags.end()) {
    attr->setSection(tags.at(eve_vtl_spec::KEY_SECTION));
  }
  if (attr->isValidAttr()) {
    vtl_attr_ = attr;
    return true;
  }
  return false;
}

std::string EveVTLInterfaceConverter::convertInfraCommand(const uint8_t& input_command) const
{
  return (input_command == InfrastructureCommand::REQUESTING) ?
    (eve_vtl_spec::VALUE_SECTION_REQ) : (eve_vtl_spec::VALUE_SECTION_NULL);
}

std::optional<std::string>
  EveVTLInterfaceConverter::convertADState(const StateMachine::ConstSharedPtr& state) const
{
  if (!vtl_attr_) {
    return std::nullopt;
  }
  const auto permit_state_opt = vtl_attr_->permitState();
  if (!permit_state_opt) {
    return eve_vtl_spec::VALUE_PERMIT_STATE_NULL;
  }
  const auto permit_state = permit_state_opt.value();

  if (!state) {
    return std::nullopt;
  }
  const auto& ctl_state = state->control_layer_state;
  const auto& srv_state = state->service_layer_state;
  bool is_valid_state = false;
  if (permit_state == eve_vtl_spec::VALUE_PERMIT_STATE_MANUAL) {
      is_valid_state = (ctl_state == StateMachine::MANUAL);
  }
  else if (permit_state == eve_vtl_spec::VALUE_PERMIT_STATE_EMERGENCY) {
    is_valid_state = (srv_state == StateMachine::STATE_EMERGENCY_STOP);
  }
  else if (permit_state == eve_vtl_spec::VALUE_PERMIT_STATE_ARRIVAL_GOAL) {
    is_valid_state = (srv_state == StateMachine::STATE_ARRIVED_GOAL);
  }
  else if (permit_state == eve_vtl_spec::VALUE_PERMIT_STATE_ENGAGE) {
    is_valid_state = (srv_state == StateMachine::STATE_INFORM_ENGAGE);
  }
  else if (permit_state == eve_vtl_spec::VALUE_PERMIT_STATE_DRIVING) {
    const bool fill_lower_bound = (srv_state >= StateMachine::STATE_RUNNING);
    const bool fill_upper_bound = (srv_state < StateMachine::STATE_ARRIVED_GOAL);
    is_valid_state = (fill_lower_bound && fill_upper_bound);
  }
  else if (permit_state == eve_vtl_spec::VALUE_PERMIT_STATE_NULL) {
    is_valid_state = true;
  }
  return (is_valid_state) ? permit_state_opt : std::nullopt;
}

}  // namespace eve_vtl_interface_converter
