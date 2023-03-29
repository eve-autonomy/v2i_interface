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

constexpr double CONVERTER_EXPIRE_SECONDS = 1.0;

IFConverterDataPipeline::IFConverterDataPipeline()
{}

void IFConverterDataPipeline::add(const std::shared_ptr<InterfaceConverterMultiMap> input)
{
  refresh();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!converter_multimap_) {
    converter_multimap_ = input;
    return;
  }
  for (const auto & [key, value] : *input) {
    auto range = converter_multimap_->equal_range(key);
    auto way_id_search_result = std::find_if(range.first, range.second, [&value](const auto & pair) {
      return pair.second->command().id == value->command().id;
    });
    if (way_id_search_result != range.second) {
      way_id_search_result->second = value;
    }
    else {
      converter_multimap_->insert({key, value});
    }
  }
}

std::shared_ptr<InterfaceConverterMultiMap> IFConverterDataPipeline::load()
{
  refresh();
  std::lock_guard<std::mutex> lock(mutex_);
  return converter_multimap_;
}

void IFConverterDataPipeline::refresh()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!converter_multimap_) {
    return;
  }
  for (auto it = converter_multimap_->begin(); it != converter_multimap_->end();) {
    auto& value = it->second;
    const auto duration = rclcpp::Clock(RCL_ROS_TIME).now() - value->command().stamp;
    if (duration.seconds() > CONVERTER_EXPIRE_SECONDS) {
      it = converter_multimap_->erase(it);
    }
    else {
      ++it;
    }
  }
}

}  // namespace interface_converter_data_pipeline
