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

# Get Integration Service dependencies
find_package(is-mock REQUIRED)
find_package(rclcpp REQUIRED)

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
    set(options NO_ROS_DEPENDENCIES)
    set(multiValueArgs SOURCE ENVIRONMENTS PREPROCESSOR)
    cmake_parse_arguments(TEST "${options}" "${uniValueArgs}" "${multiValueArgs}" ${ARGN})

    add_executable(${TEST_NAME} ${TEST_SOURCE})

    if(NOT TEST_NO_ROS_DEPENDENCIES)
        set(link_dependencies
            ${rclcpp_LIBRARIES}
            ${geometry_msgs_LIBRARIES}
            ${nav_msgs_LIBRARIES}
            ${std_msgs_LIBRARIES}
            )

        set(include_dependencies
            ${rclcpp_INCLUDE_DIRS}
            ${nav_msgs_INCLUDE_DIRS}
            ${geometry_msgs_INCLUDE_DIRS}
            ${std_msgs_INCLUDE_DIRS}
            )
    endif()

    target_link_libraries(${TEST_NAME}
        PUBLIC
            is::mock
            is-ros2-distro
            ${link_dependencies}
        PRIVATE
            $<IF:$<BOOL:${IS_GTEST_EXTERNAL_PROJECT}>,libgtest,gtest>
    	)

    target_include_directories(${TEST_NAME}
        PRIVATE
            ${include_dependencies}
        )

    target_compile_options(${TEST_NAME}
        INTERFACE
            $<$<CXX_COMPILER_ID:GNU>:-Wall>
            $<$<CXX_COMPILER_ID:GNU>:-Wextra>
        )

    if(TEST_PREPROCESSOR)
        string(REPLACE ";" " " preprocessor_definitions "${TEST_PREPROCESSOR}")
    endif()

    target_compile_definitions(${TEST_NAME}
        PRIVATE
            "ROS2__ROSIDL__BUILD_DIR=\"${CMAKE_BINARY_DIR}/is/rosidl/ros2/lib\""
            ${preprocessor_definitions}
        )

    set_target_properties(${TEST_NAME}
        PROPERTIES
            CXX_STANDARD 17
        )

    add_gtest(${TEST_NAME} SOURCES ${TEST_SOURCE})

    unset(link_dependencies)
    unset(include_dependencies)
    unset(preprocessor_definitions)
endmacro()

include(CTest)
include(${IS_GTEST_CMAKE_MODULE_DIR}/gtest.cmake)
enable_testing()

compile_test(${PROJECT_NAME}_geometry_msgs SOURCE integration/ros2__geometry_msgs.cpp)
compile_test(${PROJECT_NAME}_test_qos SOURCE integration/ros2__test_qos.cpp)

if (rclcpp_VERSION VERSION_GREATER_EQUAL 2.5)
    compile_test(${PROJECT_NAME}_test_domain SOURCE integration/ros2__test_domain.cpp)
else()
    compile_test(${PROJECT_NAME}_test_domain SOURCE integration/ros2__test_domain__foxy.cpp)
endif()

# Check if this version of ros forces wait for subscription
execute_process(
    COMMAND bash -c ". /opt/ros/${IS_ROS2_DISTRO}/setup.bash && ros2 topic pub --help"
    OUTPUT_VARIABLE ROS2_PUB_HELP
    )

if(ROS2_PUB_HELP MATCHES WAIT_MATCHING_SUBSCRIPTIONS)
    set(WAIT "WAIT=\" -w 0\"")
else()
    set(WAIT "WAIT=\"\"")
endif()

compile_test(${PROJECT_NAME}_check_ros2pkg_creation
        NO_ROS_DEPENDENCIES
        PREPROCESSOR ${WAIT}
        SOURCE integration/check_ros2pkg_creation.cpp)

# Windows dll dependencies installation
if(WIN32)
    find_file(MOCKDLL NAMES "is-mock.dll" PATHS "${is-mock_DIR}" PATH_SUFFIXES "lib" )
    install (FILES ${MOCKDLL} DESTINATION "${CMAKE_INSTALL_PREFIX}/../bin")
endif()
