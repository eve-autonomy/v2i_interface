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

#ifndef VTL_ADAPTER__EVE_VTL_ATTRIBUTE_HPP_
#define VTL_ADAPTER__EVE_VTL_ATTRIBUTE_HPP_

#include <memory>
#include <optional>

#include "vtl_adapter/eve_vtl_specification.hpp"
#include "vtl_adapter/autoware_lanelet_specification.hpp"

namespace eve_vtl_attribute
{

class EveVTLAttr
{
public:
  EveVTLAttr();
  void setType(const std::string& type);
  bool setID(const std::string& id_str);
  bool setMode(const std::string& mode);
  bool setRequestBit(const std::string& input_bit);
  bool setExpectBit(const std::string& input_bit);
  bool setTurnDirection(const std::string& turn_direction);
  bool setPermitState(const std::string& state);
  bool setResponseType(const std::string& response_type);
  bool setSection(const std::string& section);

  bool isValidAttr() const;
  const std::optional<uint8_t>& id() const;
  const std::string& type() const;
  const std::optional<std::string>& permitState() const;
  std::optional<uint8_t> request(
    const std::string& infra_cmd,
    const std::optional<std::string>& ad_state) const;
  bool response(const uint8_t& response_bit) const;

private:
  bool isValidID(const uint8_t& id) const;
  bool isValidMode(const std::string& mode) const;
  bool isValidType(const std::string& type) const;
  bool isValidBit(const uint8_t& bit) const;
  bool isValidDirection(const std::string& turn_direction) const;
  bool isValidState(const std::string& state) const;
  bool isValidSection(const std::string& section) const;
  std::optional<uint8_t> calcBit(const std::string& input_bit) const;
  std::optional<uint8_t> fixedBit(const std::string& input_bit) const;
  std::optional<uint8_t> turnDirectionBit() const;

  std::string type_;
  std::optional<uint8_t> id_;
  std::optional<std::string> response_type_;
  std::optional<std::string> mode_;
  std::optional<std::string> section_;
  std::optional<std::string> permit_state_;
  std::optional<uint8_t> request_bit_;
  std::optional<uint8_t> expect_bit_;
  std::optional<std::string> turn_direction_;
};

}  // namespace eve_vtl_attribute

#endif  // VTL_ADAPTER__EVE_VTL_ATTRIBUTE_HPP_