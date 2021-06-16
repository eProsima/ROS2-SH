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

project(is-ros2 VERSION "3.0.0" LANGUAGES CXX)

###################################################################################
# Configure options
###################################################################################
option(BUILD_LIBRARY "Compile the ROS 2 SystemHandle" ON)
set(IS_ROS2_SH_MODE "Static" CACHE STRING "Select the ROS 2 SystemHandle Mode")

###################################################################################
# Load external CMake Modules.
###################################################################################
if(BUILD_LIBRARY)
    # Check if the user specified which ROS 2 version he/she is using. If not, warn and try to automatically get it
    if(NOT IS_ROS2_DISTRO)

        message(WARNING "
            The variable 'IS_ROS2_DISTRO' was not used. You might want to set it to
            specify which ROS 2 version should be used to compile the ROS 2 System Handle.
            By default, a ROS 2 version from the sourced environment will be retrieved automatically.")

            if ("$ENV{ROS_VERSION}" STREQUAL "1")
                message(FATAL_ERROR "
                    A ROS 1 version was sourced last in your build environment.
                    Please use the 'IS_ROS2_VERSION' variable!")
            else()
                set(IS_ROS2_DISTRO $ENV{ROS_DISTRO})
            endif()

    endif()

    message(STATUS "Compiling the ${PROJECT_NAME} System Handle for ROS 2 ${IS_ROS2_DISTRO}...")

    # Generate the config.hpp file
    configure_file(${PROJECT_SOURCE_DIR}/config.hpp.in
        ${CMAKE_INSTALL_PREFIX}/include/is/sh/ros2/config.hpp
    )

    # Build ROS2 Static SH or ROS2 Dynamic SH
    set(IS_ROS2_SH_MODE_LOWERCASE "" CACHE STRING "Build mode to lowercase")
    string(TOLOWER "${IS_ROS2_SH_MODE}" IS_ROS2_SH_MODE_LOWERCASE)

    if("${IS_ROS2_SH_MODE_LOWERCASE}" STREQUAL "static")
        add_subdirectory(static)
    elseif("${IS_ROS2_SH_MODE_LOWERCASE}" STREQUAL "dynamic")
        add_subdirectory(dynamic)
    else
        message(WARNING "Invalid mode selected. Please choose between 'static' or 'dynamic'")
    endif()
endif()