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

###################################################################################
# Configure options
###################################################################################
option(BUILD_LIBRARY "Compile the ROS 2 SystemHandle" ON)

if(NOT BUILD_LIBRARY)
    return()
endif()

###################################################################################
# Load external CMake Modules.
###################################################################################
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${SANITIZERS_ROOT}/cmake)

find_package(Sanitizers QUIET)

if(SANITIZE_ADDRESS)
    message(STATUS "Preloading AddressSanitizer library could be done using \"${ASan_WRAPPER}\" to run your program.")
endif()

# identify the distro
if(ENV{ROS_DISTRO})
    set(IS_ROS2_DISTRO "ENV{ROS_DISTRO}")
else()
    find_package(ros_environment REQUIRED)

    file(GLOB_RECURSE hook_files "${ros_environment_DIR}/../*ros_distro.sh" "${ros_environment_DIR}/../*ros_distro.bat")
    file(STRINGS ${hook_files} export_statement REGEX "ROS_DISTRO=")
    string(REGEX MATCH "ROS_DISTRO=(.*)" hook_files "${export_statement}")
    unset(hook_files)
    unset(export_statement)

    set(IS_ROS2_DISTRO "${CMAKE_MATCH_1}")
endif()

# Generate the config.hpp file
configure_file(${PROJECT_SOURCE_DIR}/config.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/is/sh/ros2/config.hpp
    )

include(GNUInstallDirs)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/is/sh/ros2/config.hpp
    DESTINATION	${CMAKE_INSTALL_INCLUDEDIR}/is/sh/ros2
    )

# Use an interface target to distribute the distro dependency
add_library(is-ros2-distro INTERFACE)

target_include_directories(is-ros2-distro
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )

# install the headers
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

###################################################################################
# External dependencies for the Integration Service ROS 2 SystemHandle library
###################################################################################
find_package(is-core REQUIRED)
find_package(fastrtps REQUIRED)

###################################################################################
# Configure the Integration Service ROS 2 SystemHandle library
###################################################################################
add_library(${PROJECT_NAME}
    SHARED
        src/Conversion.cpp
        src/Participant.cpp
        src/Publisher.cpp
        src/Subscriber.cpp
        src/SystemHandle.cpp
)

if (Sanitizers_FOUND)
    add_sanitizers(${PROJECT_NAME})
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION
        ${PROJECT_VERSION}
    SOVERSION
        ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    CXX_STANDARD
        17
    CXX_STANDARD_REQUIRED
        YES
    )

target_compile_options(${PROJECT_NAME}
    PRIVATE
        $<$<CXX_COMPILER_ID:GNU>:-pedantic>
        $<$<CXX_COMPILER_ID:GNU>:-fstrict-aliasing>
        $<$<CXX_COMPILER_ID:GNU>:-Wall>
        $<$<CXX_COMPILER_ID:GNU>:-Wextra>
        $<$<CXX_COMPILER_ID:GNU>:-Wcast-align>
        $<$<CXX_COMPILER_ID:GNU>:-Wshadow>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
        $<$<CXX_COMPILER_ID:MSVC>:/wd4700>
        $<$<CXX_COMPILER_ID:MSVC>:/wd4996>
        $<$<CXX_COMPILER_ID:MSVC>:/wd4820>
        $<$<CXX_COMPILER_ID:MSVC>:/wd4255>
        $<$<CXX_COMPILER_ID:MSVC>:/wd4668>
    )

include(GNUInstallDirs)
message(STATUS "Configuring [${PROJECT_NAME}]...")

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        is::core
        fastrtps
        is-ros2-distro
    )

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    )

###################################################################################
# Install the Integration Service ROS 2 SystemHandle library
###################################################################################
is_install_middleware_plugin(
    MIDDLEWARE
        ros2_dynamic
    TARGET
        ${PROJECT_NAME}
    INTERFACES
        is-ros2-distro
    )

file(
    COPY
        ${CMAKE_CURRENT_LIST_DIR}/resources/
    DESTINATION
        /tmp/is_ros2_sh
)

###################################################################################
# Integration Service ROS 2 SystemHandle tests
###################################################################################
if(BUILD_TESTS OR BUILD_ROS2_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
