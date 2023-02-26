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

#include <iostream>
#include <memory>
#include <optional>
#include <algorithm>
#include "vtl_adapter/eve_vtl_attribute.hpp"

namespace eve_vtl_attribute
{
using namespace eve_vtl_spec;
using namespace aw_lanelet_spec;

/*
*************************************************************
  Local function
*************************************************************
*/
std::optional<uint8_t> stringToInt(const std::string& input_str)
{
  uint8_t output_int;
  try {
    output_int = std::stoi(input_str);
  }
  catch (const std::invalid_argument& ex) {
    std::cerr << ex.what() << std::endl;
    return std::nullopt;
  }
  catch (const std::out_of_range& ex) {
    std::cerr << ex.what() << std::endl;
    return std::nullopt;
  }
  return output_int;
}

std::string toUpper(const std::string& str)
{
  std::string upper_str(str);
  std::transform(upper_str.begin(), upper_str.end(), upper_str.begin(),
    [](unsigned char ch){ return std::toupper(ch); });
  return upper_str;
}

/*
*************************************************************
  Class public function
*************************************************************
*/

EveVTLAttr::EveVTLAttr() :
  id_(std::nullopt),
  response_type_(std::nullopt),
  mode_(std::nullopt),
  section_(std::string()),
  permit_state_(std::string()),
  request_bit_(MAX_VALUE_BIT),
  expect_bit_(MAX_VALUE_BIT),
  turn_direction_(std::nullopt)
{
}

void EveVTLAttr::setType(const std::string& type)
{
  type_ = type;
}

bool EveVTLAttr::setID(const std::string& id_str)
{
  id_ = stringToInt(id_str);
  if (!id_) {
    return false;
  }
  return !isValidID(id_.value());
}

bool EveVTLAttr::setMode(const std::string& mode)
{
  const auto mode_upper = toUpper(mode);
  if(!isValidMode(mode_upper)) {
    mode_ = std::nullopt;
    return false;
  }
  mode_ = mode_upper;
  return true;
}

bool EveVTLAttr::setRequestBit(const std::string& input_bit)
{
  request_bit_ = calcBit(input_bit);
  if (!request_bit_) {
    return false;
  }
  return isValidBit(request_bit_.value());
}

bool EveVTLAttr::setExpectBit(const std::string& input_bit)
{
  expect_bit_ = calcBit(input_bit);
  if (!expect_bit_) {
    return false;
  }
  return isValidBit(expect_bit_.value());
}

bool EveVTLAttr::setTurnDirection(const std::string& turn_direction)
{
  if (!isValidDirection(turn_direction)) {
    turn_direction_ = std::nullopt;
    return false;
  }
  turn_direction_ = turn_direction;
  return true;
}

bool EveVTLAttr::setPermitState(const std::string& state)
{
  const auto state_upper = toUpper(state);
  if (!isValidState(state_upper)) {
    permit_state_ = std::nullopt;
    return false;
  }
  permit_state_ = state_upper;
  return true;
}

bool EveVTLAttr::setResponseType(const std::string& response_type)
{
  const auto response_type_upper = toUpper(response_type);
  if (!isValidType(response_type_upper)) {
    response_type_ = std::nullopt;
    return false;
  }
  response_type_ = response_type_upper;
  return true;
}

bool EveVTLAttr::setSection(const std::string& section)
{
  const auto section_upper = toUpper(section);
  if (!isValidSection(section_upper)) {
    section_ = std::nullopt;
    return false;
  }
  section_ = section_upper;
  return true;
}

bool EveVTLAttr::isValidAttr() const
{
  if (!id_.has_value() ||
    !response_type_.has_value() ||
    !mode_.has_value() ||
    !section_.has_value() ||
    !permit_state_.has_value() ||
    !request_bit_.has_value() ||
    !expect_bit_.has_value()) {
    return false;
  }
  return 
    mode_ != VALUE_MODE_TURN_DIRECTION ||
    turn_direction_.has_value();
}

const std::optional<uint8_t>& EveVTLAttr::id() const
{
  return id_;
}

const std::string& EveVTLAttr::type() const
{
  return type_;
}

const std::optional<std::string>& EveVTLAttr::permitState() const
{
  return permit_state_;
}

std::optional<uint8_t> EveVTLAttr::request(
  const std::string& infra_cmd,
  const std::optional<std::string>& ad_state) const
{
  if (!section_ || !permit_state_) {
    return std::nullopt;
  }
  const auto sec = section_.value();
  const auto state = permit_state_.value();
  if (!isValidAttr()) {
    return std::nullopt;
  }
  if ((sec != VALUE_SECTION_NULL) && (sec != infra_cmd)) {
    return std::nullopt;
  }
  if (state != VALUE_PERMIT_STATE_NULL) {
    if (!ad_state) {
      return std::nullopt;
    }
    if (state != ad_state.value()) {
      return std::nullopt;
    }
  }
  return request_bit_;
}

bool EveVTLAttr::response(const uint8_t& response_bit) const
{
  if (!isValidAttr()) {
    return false;
  }
  if (!response_type_ || !expect_bit_) {
    return false;
  }
  const auto type = response_type_.value();
  const auto expect_bit = expect_bit_.value();
  return
    (type == VALUE_RESPONSE_TYPE_ALWAYS) ? (true) :
    (type == VALUE_RESPONSE_TYPE_AND) ? ((response_bit & expect_bit) != 0) :
    (type == VALUE_RESPONSE_TYPE_MATCH) ? (response_bit == expect_bit) :
    (false);
}

/*
*************************************************************
  Class private function
*************************************************************
*/

bool EveVTLAttr::isValidID(const uint8_t& id) const
{
  return (id >= MIN_VALUE_ID && id <= MAX_VALUE_ID); 
}

bool EveVTLAttr::isValidMode(const std::string& mode) const
{
  return
    (mode == VALUE_MODE_FIXED) ||
    (mode == VALUE_MODE_TURN_DIRECTION);
}

bool EveVTLAttr::isValidType(const std::string& type) const
{
  return
    (type == VALUE_RESPONSE_TYPE_ALWAYS) ||
    (type == VALUE_RESPONSE_TYPE_AND) ||
    (type == VALUE_RESPONSE_TYPE_MATCH);
}

bool EveVTLAttr::isValidBit(const uint8_t& bit) const
{
  return (bit < MIN_VALUE_BIT || bit > MAX_VALUE_BIT);
}

bool EveVTLAttr::isValidDirection(const std::string& turn_direction) const
{
  return
    (turn_direction == VALUE_TURN_DIRECTION_STRAIGHT) ||
    (turn_direction == VALUE_TURN_DIRECTION_RIGHT) ||
    (turn_direction == VALUE_TURN_DIRECTION_LEFT);
}

bool EveVTLAttr::isValidState(const std::string& state) const
{
  return
    (state == VALUE_PERMIT_STATE_DRIVING) ||
    (state == VALUE_PERMIT_STATE_MANUAL) ||
    (state == VALUE_PERMIT_STATE_EMERGENCY) ||
    (state == VALUE_PERMIT_STATE_ARRIVAL_GOAL) ||
    (state == VALUE_PERMIT_STATE_ENGAGE) ||
    (state == VALUE_PERMIT_STATE_NULL);
}

bool EveVTLAttr::isValidSection(const std::string& section) const
{
  return
    (section == VALUE_SECTION_REQ) ||
    (section == VALUE_SECTION_NULL);
}

std::optional<uint8_t> EveVTLAttr::calcBit(const std::string& input_bit) const
{
  return
    (!response_type_) ? std::nullopt :
    (response_type_ == VALUE_RESPONSE_TYPE_ALWAYS) ? MAX_VALUE_BIT :
    (mode_ == VALUE_MODE_FIXED) ? fixedBit(input_bit) :
    (mode_ == VALUE_MODE_TURN_DIRECTION) ? turnDirectionBit() :
    std::nullopt;
}

std::optional<uint8_t> EveVTLAttr::fixedBit(const std::string& input_bit) const
{
  const auto bit = stringToInt(input_bit);
  if (!bit) {
    return std::nullopt;
  }
  return isValidBit(bit.value()) ? bit : std::nullopt;
}

std::optional<uint8_t> EveVTLAttr::turnDirectionBit() const
{
  const TD_BIT td_bit = 
    (turn_direction_ == VALUE_TURN_DIRECTION_STRAIGHT) ? (TD_BIT::STRAIGHT) :
    (turn_direction_ == VALUE_TURN_DIRECTION_RIGHT) ? (TD_BIT::RIGHT) :
    (turn_direction_ == VALUE_TURN_DIRECTION_LEFT) ? (TD_BIT::LEFT) :
    (TD_BIT::INVALID);
  const uint8_t bit = static_cast<uint8_t>(td_bit);
  return isValidBit(bit) ? std::optional<uint8_t>(bit) : std::nullopt;
}

}  // namespace eve_vtl_attribute
