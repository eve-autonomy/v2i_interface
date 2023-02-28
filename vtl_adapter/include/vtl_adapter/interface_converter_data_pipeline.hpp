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

#ifndef VTL_ADAPTER__INTERFACE_CONVERTER_DATA_PIPELINE_HPP_
#define VTL_ADAPTER__INTERFACE_CONVERTER_DATA_PIPELINE_HPP_

#include <mutex>
#include "vtl_adapter/eve_vtl_interface_converter.hpp"

namespace interface_converter_data_pipeline
{
using InterfaceConverter = eve_vtl_interface_converter::EveVTLInterfaceConverter;
using InterfaceConverterMap =
  std::unordered_map<uint8_t, std::shared_ptr<InterfaceConverter>>;

class IFConverterDataPipeline
{
public:
  IFConverterDataPipeline();
  void add(const std::shared_ptr<InterfaceConverterMap> input);
  std::shared_ptr<InterfaceConverterMap> load();
private:
  std::shared_ptr<InterfaceConverterMap> converter_map_;
  std::mutex mutex_;
};

}  // namespace interface_converter_data_pipeline

#endif  // VTL_ADAPTER__INTERFACE_CONVERTER_DATA_PIPELINE_HPP_
