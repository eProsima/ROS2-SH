# Copyright 2019 Open Source Robotics Foundation, Inc.
# Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
#
#  library and integration-service executable CMake project

##################################################################################
# CMake build rules for the Integration Service ROS 2 SystemHandle library
##################################################################################
cmake_minimum_required(VERSION 3.5.0 FATAL_ERROR)

# IS_PACKAGE_NAME = Project name

project(${IS_PACKAGE_NAME})

foreach(PKG ${PACKAGE_DEPENDENCIES})
        find_package(${PKG} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB_RECURSE IDL_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.idl")

rosidl_generate_interfaces(${IS_PACKAGE_NAME}
  ${IDL_FILES}
  DEPENDENCIES ${PACKAGE_DEPENDENCIES}
  ""
)

ament_package()