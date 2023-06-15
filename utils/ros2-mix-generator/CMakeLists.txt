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
# CMake project to help generating middleware interface extension files for specified ROS types

###############################################################################################
# CMake build rules for the Integration Service ROS 2 pkg MIX generator library
###############################################################################################
cmake_minimum_required(VERSION 3.5.0)

project(is-ros2-mix-generator)

###################################################################################
# Configure options
###################################################################################
option(BUILD_LIBRARY "Compile the ROS 2 SystemHandle" ON)
set(IS_ROS2_SH_MODE "Static" CACHE STRING "Select the ROS 2 SystemHandle Mode")

if(NOT BUILD_LIBRARY)
    return()
endif()

set(IS_ROS2_SH_MODE_LOWERCASE "" CACHE STRING "Build mode to lowercase")
string(TOLOWER "${IS_ROS2_SH_MODE}" IS_ROS2_SH_MODE_LOWERCASE)

if("${IS_ROS2_SH_MODE_LOWERCASE}" STREQUAL "dynamic")
    return()
endif()

# Get dependencies
find_package(is-core REQUIRED)
find_package(rosidl_parser REQUIRED)
find_package(is-ros2 REQUIRED)
find_package(Python3 REQUIRED)

list(APPEND MIX_ROS_PACKAGES_LIST std_msgs)

if (MIX_ROS_PACKAGES)

    string(REPLACE " " ";" MIX_ROS_PACKAGES_ ${MIX_ROS_PACKAGES})

    foreach(PKG ${MIX_ROS_PACKAGES_})
        find_package(${PKG} REQUIRED)

        list(APPEND MIX_ROS_PACKAGES_LIST ${PKG})
    endforeach()

endif()

if (MIX_ROS2_PACKAGES)

    string(REPLACE " " ";" MIX_ROS2_PACKAGES_ ${MIX_ROS2_PACKAGES})

    foreach(PKG ${MIX_ROS2_PACKAGES_})
        find_package(${PKG} REQUIRED)

        list(APPEND MIX_ROS_PACKAGES_LIST ${PKG})
    endforeach()

endif()

if (BUILD_TESTS OR BUILD_ROS2_TESTS)

    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)

    list(APPEND MIX_ROS_PACKAGES_LIST geometry_msgs nav_msgs)

endif()

###################################################################################
# Install the Integration Service is-ros2-rosidl-mix plugin
###################################################################################
is_mix_install_extension(
    IDL_TYPE rosidl
    MIDDLEWARE
        ros2
    MESSAGE
        SOURCES "${CMAKE_CURRENT_LIST_DIR}/resources/convert__msg.cpp.em"
        HEADERS "${CMAKE_CURRENT_LIST_DIR}/resources/convert__msg.hpp.em"
    SERVICE
        SOURCES "${CMAKE_CURRENT_LIST_DIR}/resources/convert__srv.cpp.em"
)

include(cmake/is_ros2_rosidl_mix.cmake)

is_ros2_rosidl_mix(
    PACKAGES ${MIX_ROS_PACKAGES_LIST}
    MIDDLEWARES ros2
)

# Install Config.cmake file
install(
    FILES
        ${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Config.cmake
    DESTINATION
        "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)
