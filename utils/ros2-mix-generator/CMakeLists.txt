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

# Get dependencies
find_package(is-ros2-rosidl REQUIRED)
find_package(is-ros2 REQUIRED)

list(APPEND MIX_ROS_PACKAGES_LIST std_msgs)

if (MIX_ROS_PACKAGES)

    string(REPLACE " " ";" MIX_ROS_PACKAGES_ ${MIX_ROS_PACKAGES})

    foreach(PKG ${MIX_ROS_PACKAGES_})
        find_package(${PKG} REQUIRED)

        list(APPEND MIX_ROS_PACKAGES_LIST ${PKG})
    endforeach()

endif()

if (BUILD_TESTS OR BUILD_ROS2_TESTS)

    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(is_ros2_test_msg REQUIRED)

    list(APPEND MIX_ROS_PACKAGES_LIST geometry_msgs nav_msgs is_ros2_test_msg)

endif()

is_ros2_rosidl_mix(
    PACKAGES ${MIX_ROS_PACKAGES_LIST}
    MIDDLEWARES ros2
)

find_program(NODERED_EXECUTABLE NAMES node-red node-red@ PATH_SUFFIXES bin)

if(NODERED_EXECUTABLE)
    file(WRITE $ENV{HOME}/rosidl_msgs_path.txt "${CMAKE_INSTALL_PREFIX}/lib/is/ros2/msg\n${CMAKE_INSTALL_PREFIX}/include/is/rosidl/ros2")
endif()

# Install Config.cmake file
install(
    FILES
        ${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Config.cmake
    DESTINATION
        "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)
