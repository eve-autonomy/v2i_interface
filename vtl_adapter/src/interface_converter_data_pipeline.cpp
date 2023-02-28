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

#include "vtl_adapter/interface_converter_data_pipeline.hpp"

namespace interface_converter_data_pipeline
{

IFConverterDataPipeline::IFConverterDataPipeline()
{}

void IFConverterDataPipeline::add(const std::shared_ptr<InterfaceConverterMap> input)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!converter_map_) {
    converter_map_ = input;
    return;
  }
  input->merge(*converter_map_);
  converter_map_->swap(*input);
}

std::shared_ptr<InterfaceConverterMap> IFConverterDataPipeline::load()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return converter_map_;
}

}  // namespace interface_converter_data_pipeline
