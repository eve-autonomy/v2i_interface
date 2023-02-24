cmake_minimum_required(VERSION 3.14)
project(v2i_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(autoware_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
autoware_package()
ament_auto_find_build_dependencies()

ament_auto_add_library(vtl_command_converter SHARED
  src/vtl_command_converter.cpp
  src/eve_vtl_interface_converter.cpp
  src/eve_vtl_attribute.cpp
)

rclcpp_components_register_node(vtl_command_converter
  PLUGIN "vtl_command_converter::VtlCommandConverterNode"
  EXECUTABLE vtl_command_converter_node
)

#ament_target_dependencies(vtl_command_converter
#  rclcpp
#  tier4_v2x_msgs
#  v2i_interface_msgs
#)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(scripts)
install(
  PROGRAMS
  scripts/v2i_interface.py
  scripts/test/v2i_interface_test.py
  DESTINATION
  lib/${PROJECT_NAME})

install(FILES DESTINATION share/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
