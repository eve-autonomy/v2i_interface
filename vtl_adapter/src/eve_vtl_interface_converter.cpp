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


#include "vtl_adapter/eve_vtl_interface_converter.hpp"
#include "vtl_adapter/eve_vtl_specification.hpp"
#include "vtl_adapter/autoware_lanelet_specification.hpp"

namespace eve_vtl_interface_converter
{

constexpr static unsigned int ERROR_THROTTLE_MSEC = 1000;

/*
***************************************************************
Class public function
***************************************************************
*/

EveVTLInterfaceConverter::EveVTLInterfaceConverter(
  const InfrastructureCommand& input_command, rclcpp::Node* node)
  : command_(input_command), node_(node)
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
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "EveVTLInterfaceConverter::%s: vtl_attr_ is not initialized", __func__);
    return std::nullopt;
  }
  const auto command_str = convertInfraCommand(command_.state);
  const auto state_str = convertADState(state);
  return vtl_attr_->request(command_str, state_str);
}

bool EveVTLInterfaceConverter::response(const uint8_t& response_bit) const
{
  if (!vtl_attr_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "EveVTLInterfaceConverter::%s: vtl_attr_ is not initialized", __func__);
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
    RCLCPP_DEBUG(
      node_->get_logger(), "EveVTLInterfaceConverter::%s: type is invalid: %s",
      __func__, type.c_str());
    return false;
  }

  const auto& custom_tags = input_command.custom_tags;
  // custom_tagsが設定されてなければ初期化不要（失敗）
  if (custom_tags.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "EveVTLInterfaceConverter::%s: custom_tags is empty", __func__);
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
    const bool ret =
      attr->setTurnDirection(tags.at(aw_lanelet_spec::KEY_TURN_DIRECTION));
    if (!ret) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
        "EveVTLInterfaceConverter::%s: setTurnDirection calculation failed: input=%s",
        __func__, tags.at(aw_lanelet_spec::KEY_TURN_DIRECTION).c_str());
    }
  }
  else {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "EveVTLInterfaceConverter::%s: turn_direction is not set", __func__);
  }

  if (tags.find(eve_vtl_spec::KEY_MODE) != tags.end()) {
    const bool ret =
      attr->setMode(tags.at(eve_vtl_spec::KEY_MODE));
    if (!ret) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
        "EveVTLInterfaceConverter::%s: setMode calculation failed: input=%s",
        __func__, tags.at(eve_vtl_spec::KEY_MODE).c_str());
    }
  }
  else {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "EveVTLInterfaceConverter::%s: mode is not set", __func__);
  }

  if (tags.find(eve_vtl_spec::KEY_ID) != tags.end()) {
    const bool ret =
      attr->setID(tags.at(eve_vtl_spec::KEY_ID));
    if (!ret) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
        "EveVTLInterfaceConverter::%s: setID calculation failed: input=%s",
        __func__, tags.at(eve_vtl_spec::KEY_ID).c_str());
    }
  }
  else {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "EveVTLInterfaceConverter::%s: id is not set", __func__);
  }

  if (tags.find(eve_vtl_spec::KEY_RESPONSE_TYPE) != tags.end()) {
    const bool ret =
      attr->setResponseType(tags.at(eve_vtl_spec::KEY_RESPONSE_TYPE));
    if (!ret) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
        "EveVTLInterfaceConverter::%s: setResponseType calculation failed: input=%s",
        __func__, tags.at(eve_vtl_spec::KEY_RESPONSE_TYPE).c_str());
    }
  }
  else {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "EveVTLInterfaceConverter::%s: response_type is not set", __func__);
  }

  if (tags.find(eve_vtl_spec::KEY_REQUEST_BIT) != tags.end()) {
    const bool ret =
      attr->setRequestBit(tags.at(eve_vtl_spec::KEY_REQUEST_BIT));
    if (!ret) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
        "EveVTLInterfaceConverter::%s: setRequestBit calculation failed: input=%s",
        __func__, tags.at(eve_vtl_spec::KEY_REQUEST_BIT).c_str());
    }
  }
  else {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "EveVTLInterfaceConverter::%s: request_bit is not set", __func__);
  }

  if (tags.find(eve_vtl_spec::KEY_EXPECT_BIT) != tags.end()) {
    const bool ret =
      attr->setExpectBit(tags.at(eve_vtl_spec::KEY_EXPECT_BIT));
    if (!ret) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
        "EveVTLInterfaceConverter::%s: setExpectBit calculation failed: input=%s",
        __func__, tags.at(eve_vtl_spec::KEY_EXPECT_BIT).c_str());
    }
  }
  else {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "EveVTLInterfaceConverter::%s: expect_bit is not set", __func__);
  }

  if (tags.find(eve_vtl_spec::KEY_PERMIT_STATE) != tags.end()) {
    const bool ret =
      attr->setPermitState(tags.at(eve_vtl_spec::KEY_PERMIT_STATE));
    if (!ret) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
        "EveVTLInterfaceConverter::%s: setPermitState calculation failed: input=%s",
        __func__, tags.at(eve_vtl_spec::KEY_PERMIT_STATE).c_str());
    }
  }
  else {
    RCLCPP_DEBUG(node_->get_logger(),
      "EveVTLInterfaceConverter::%s: permit_state is not set", __func__);
  }

  if (tags.find(eve_vtl_spec::KEY_SECTION) != tags.end()) {
    const bool ret =
      attr->setSection(tags.at(eve_vtl_spec::KEY_SECTION));
    if (!ret) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
        "EveVTLInterfaceConverter::%s: setSection calculation failed: input=%s",
        __func__, tags.at(eve_vtl_spec::KEY_SECTION).c_str());
    }
  }
  else {
    RCLCPP_DEBUG(node_->get_logger(),
      "EveVTLInterfaceConverter::%s: section is not set", __func__);
  }

  if (attr->isValidAttr()) {
    vtl_attr_ = attr;
    return true;
  }
  RCLCPP_WARN_THROTTLE(
    node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
    "EveVTLInterfaceConverter::%s: attribute is invalid", __func__);
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
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "EveVTLInterfaceConverter::%s: vtl_attr_ is null", __func__);
    return std::nullopt;
  }
  const auto permit_state_opt = vtl_attr_->permitState();
  if (!permit_state_opt) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "EveVTLInterfaceConverter::%s: permit_state is null", __func__);
    return eve_vtl_spec::VALUE_PERMIT_STATE_NULL;
  }
  const auto permit_state = permit_state_opt.value();

  if (!state) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "EveVTLInterfaceConverter::%s: state is null", __func__);
    return std::nullopt;
  }
  const auto& srv_state = state->service_layer_state;
  bool is_valid_state = false;
  if (permit_state == eve_vtl_spec::VALUE_PERMIT_STATE_DRIVING) {
    const bool fill_lower_bound = (srv_state >= StateMachine::STATE_RUNNING);
    const bool fill_upper_bound = (srv_state < StateMachine::STATE_ARRIVED_GOAL);
    is_valid_state = (fill_lower_bound && fill_upper_bound);
  }
  else if (permit_state == eve_vtl_spec::VALUE_PERMIT_STATE_NULL) {
    is_valid_state = true;
  }

  if (!is_valid_state) {
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ERROR_THROTTLE_MSEC,
      "EveVTLInterfaceConverter::" << __func__ <<
      ": state is invalid: " << srv_state);
  }
  return (is_valid_state) ? permit_state_opt : std::nullopt;
}

}  // namespace eve_vtl_interface_converter
