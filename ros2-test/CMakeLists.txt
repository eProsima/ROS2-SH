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
# is-ros2 test SystemHandle library and integration-service executable CMake project

###############################################################################################
# CMake build rules for the Integration Service ROS 2 SystemHandle integration test library
###############################################################################################

cmake_minimum_required(VERSION 3.5.0)

###################################################################################
# Configure options
###################################################################################
option(BUILD_LIBRARY "Compile the ROS 2 SystemHandle" ON)

project(is-ros2-test)

if(NOT BUILD_LIBRARY)
    return()
endif()

if (NOT (BUILD_TESTS OR BUILD_ROS2_TESTS))
    return()
endif()

# Check if the user specified which ROS 2 version he/she is using. If not, warn and try to automatically get it
if(NOT IS_ROS2_DISTRO)

    message(WARNING "
        The variable 'IS_ROS2_DISTRO' was not used. You might want to set it to
        specify which ROS 2 version should be used to compile the ROS 2 System Handle.
        By default, a ROS 2 version from the sourced environment will be retrieved automatically.")

        if ("$ENV{ROS_VERSION}" STREQUAL "1")
            message(FATAL_ERROR "
                A ROS 1 version was sourced last in your build environment.
                Please consider using the 'IS_ROS2_DISTRO' variable, or source a ROS 2 version before continuing!")
        else()
            set(IS_ROS2_DISTRO $ENV{ROS_DISTRO})
        endif()

endif()

message(STATUS "Compiling the ${PROJECT_NAME} System Handle for ROS 2 ${IS_ROS2_DISTRO}...")

# Get Integration Service dependencies
find_package(xtypes REQUIRED)
find_package(is-core REQUIRED)
find_package(is-mock REQUIRED)
find_package(is-ros2 REQUIRED)
find_package(is-ros2-mix-generator REQUIRED)

# Get message dependencies
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

macro(compile_test)
    # Parse arguments
    if("${ARGV0}" STREQUAL "NAME")
        set(uniValueArgs NAME)
    else()
        set(TEST_NAME "${ARGV0}")
    endif()
    set(multiValueArgs SOURCE)
    cmake_parse_arguments(TEST "" "${uniValueArgs}" "${multiValueArgs}" ${ARGN})

    add_executable(${TEST_NAME} ${TEST_SOURCE})

    target_link_libraries(${TEST_NAME}
        PUBLIC
            is::mock
            is::ros2
            ${geometry_msgs_LIBRARIES}
            ${nav_msgs_LIBRARIES}
            ${std_msgs_LIBRARIES}
        PRIVATE
            $<IF:$<BOOL:${IS_GTEST_EXTERNAL_PROJECT}>,libgtest,gtest>
    	)

    target_include_directories(${TEST_NAME}
        PRIVATE
            ${geometry_msgs_INCLUDE_DIRS}
            ${nav_msgs_INCLUDE_DIRS}
            ${std_msgs_INCLUDE_DIRS}
        )

    target_compile_options(${TEST_NAME}
        INTERFACE
            $<$<CXX_COMPILER_ID:GNU>:-Wall>
            $<$<CXX_COMPILER_ID:GNU>:-Wextra>
        )

    set_target_properties(${TEST_NAME}
        PROPERTIES
            CXX_STANDARD 17
        )

    add_gtest(${TEST_NAME} SOURCES ${TEST_SOURCE})
endmacro()

include(CTest)
include(${IS_GTEST_CMAKE_MODULE_DIR}/gtest.cmake)
enable_testing()

compile_test(${PROJECT_NAME}_geometry_msgs SOURCE integration/ros2__geometry_msgs.cpp)
compile_test(${PROJECT_NAME}_primitive_msgs SOURCE integration/ros2__primitives_msgs.cpp)

if ("${IS_ROS2_DISTRO}" STREQUAL "foxy")
    compile_test(${PROJECT_NAME}_test_domain SOURCE integration/ros2__test_domain__foxy.cpp)
else()
    compile_test(${PROJECT_NAME}_test_domain SOURCE integration/ros2__test_domain.cpp)
endif()

set(test_msgs_config_file "ros2__test_msgs.yaml")

set_property(
    TARGET ${PROJECT_NAME}_geometry_msgs ${PROJECT_NAME}_test_domain ${PROJECT_NAME}_primitive_msgs
    APPEND PROPERTY COMPILE_DEFINITIONS PRIVATE
        "ROS2__GEOMETRY_MSGS__TEST_CONFIG=\"${CMAKE_CURRENT_LIST_DIR}/resources/ros2__geometry_msgs.yaml\""
        "ROS2__TEST_DOMAIN__TEST_CONFIG=\"${CMAKE_CURRENT_LIST_DIR}/resources/ros2__domain_change.yaml\""
        "ROS2__ROSIDL__BUILD_DIR=\"${CMAKE_BINARY_DIR}/is/rosidl/ros2/lib\""
    )

# Windows dll dependencies installation
if(WIN32)
    find_file(MOCKDLL NAMES "is-mock.dll" PATHS "${is-mock_DIR}" PATH_SUFFIXES "lib" )
    install (FILES ${MOCKDLL} DESTINATION "${CMAKE_INSTALL_PREFIX}/../bin")
endif()
