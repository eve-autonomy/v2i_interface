# Copyright 2023 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(vtl_adapter)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Generate exe file
ament_auto_add_library(vtl_adapter
  SHARED
  src/vtl_adapter.cpp
  src/vtl_state_converter.cpp
  src/vtl_command_converter.cpp
  src/interface_converter_data_pipeline.cpp
  src/eve_vtl_interface_converter.cpp
  src/eve_vtl_attribute.cpp
)

rclcpp_components_register_node(vtl_adapter
  PLUGIN "vtl_adapter::VtlAdapterNode"
  EXECUTABLE vtl_adapter_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # ament_add_gtest(test_vtl_command_converter
  #   test/src/test_**.cpp
  #   test/src/test_**.cpp
  # )
  # ament_target_dependencies(test_vtl_command_converter
  #   rclcpp
  #   tier4_v2x_msgs
  #   v2i_interface_msgs
  # )
  # target_link_libraries(test_vtl_command_converter
  #   vtl_command_converter
  # )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
