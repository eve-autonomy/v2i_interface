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

#ament_auto_add_library(** SHARED
#  src/**.cpp
#)

#rclcpp_components_register_node(**
#  PLUGIN ""
#  EXECUTABLE lateral_error_publisher_node
#)

ament_python_install_package(scripts)
install(
  PROGRAMS
  scripts/v2i_interface.py
  DESTINATION
  lib/${PROJECT_NAME})

install(FILES DESTINATION share/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
