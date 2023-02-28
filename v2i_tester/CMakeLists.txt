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
project(v2i_tester)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

## Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Generate exe file
#ament_auto_add_library(**
#  SHARED
#  src/**
#)

#rclcpp_components_register_node(**
#  PLUGIN "**"
#  EXECUTABLE *****
#)

file(GLOB SCRIPT_FILES scripts/*.bash)
file(GLOB ENV_FILES config/*.env)
install(
  PROGRAMS
  ${SCRIPT_FILES}
  ${ENV_FILES}
  DESTINATION
  lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # ament_add_gtest(**
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
  # launch
)
