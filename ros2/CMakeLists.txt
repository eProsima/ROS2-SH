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

project(is-ros2 VERSION "3.1.0" LANGUAGES CXX)

###################################################################################
# Configure options
###################################################################################
option(BUILD_LIBRARY "Compile the ROS 2 SystemHandle" ON)
set(IS_ROS2_SH_MODE "Static" CACHE STRING "Select the ROS 2 SystemHandle Mode")

###################################################################################
# Build ROS2 Static SH or ROS2 Dynamic SH
###################################################################################
set(IS_ROS2_SH_MODE_LOWERCASE "" CACHE STRING "Build mode to lowercase")
string(TOLOWER "${IS_ROS2_SH_MODE}" IS_ROS2_SH_MODE_LOWERCASE)

if("${IS_ROS2_SH_MODE_LOWERCASE}" STREQUAL "static")
    add_subdirectory(static)
    if(BUILD_TESTS OR BUILD_ROS2_TESTS)
        enable_testing()
    endif()
elseif("${IS_ROS2_SH_MODE_LOWERCASE}" STREQUAL "dynamic")
    add_subdirectory(dynamic)
    if(BUILD_TESTS OR BUILD_ROS2_TESTS)
        enable_testing()
    endif()
else()
    message(WARNING "Invalid mode selected. Please choose between 'static' or 'dynamic'")
endif()

###################################################################################
# Integration Service ROS 2 SystemHandle API Reference
###################################################################################
if(BUILD_API_REFERENCE)
    if(NOT BUILD_LIBRARY)
        find_file(IS_DOXYGEN_CONFIG_FILE doxygen-config.in PATHS ${CMAKE_INSTALL_PREFIX}/../is-core)
    endif()
    find_package(Doxygen REQUIRED)
    # Create doxygen directories
    add_custom_target(doc-dirs
        COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/doxygen
        COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/doxygen/html
        COMMENT "Creating documentation directories" VERBATIM)

    set(IS_ROS2_STATIC_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/static/include/is/sh/ros2")
    set(IS_ROS2_STATIC_SOURCE_DIR "${CMAKE_SOURCE_DIR}/static/src")
    set(IS_ROS2_DYNAMIC_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/dynamic/include/is/sh/ros2")
    file(GLOB_RECURSE HPP_FILES
        "${IS_ROS2_STATIC_INCLUDE_DIR}/*.h*"
        "${IS_ROS2_STATIC_SOURCE_DIR}/*.h*"
        "${IS_ROS2_DYNAMIC_INCLUDE_DIR}/*.h*")

    # Doxygen related variables
    set(DOXYGEN_INPUT_DIR "${IS_ROS2_STATIC_INCLUDE_DIR} ${IS_ROS2_STATIC_SOURCE_DIR} ${IS_ROS2_DYNAMIC_INCLUDE_DIR}")
    set(DOXYGEN_OUTPUT_DIR "${PROJECT_BINARY_DIR}/doxygen")
    set(DOXYGEN_INDEX_FILE "${PROJECT_BINARY_DIR}/doxygen/xml/index.xml")
    set(DOXYFILE_IN "${IS_DOXYGEN_CONFIG_FILE}")
    set(DOXYFILE_OUT ${PROJECT_BINARY_DIR}/doxygen-config)
    set(DOXYGEN_TAGFILES "../is-core/is_core.tag=../is-core/doxygen/html")
    set(DOXYGEN_HTML_DIR "${PROJECT_BINARY_DIR}/doxygen/html")

    # Configure doxygen
    configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

    # Doxygen command
    add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
        DEPENDS ${HPP_FILES}
        MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
        COMMENT "Generating doxygen documentation")

    # Generate API reference
    add_custom_target(doxygen-${PROJECT_NAME} ALL
        DEPENDS ${DOXYGEN_INDEX_FILE}
        COMMENT "Generated API documentation with doxygen" VERBATIM)
    add_dependencies(doxygen-${PROJECT_NAME} doc-dirs)

    # Install doxygen generated XML files
    install(DIRECTORY ${PROJECT_BINARY_DIR}/doxygen/xml
        DESTINATION doxygen
        COMPONENT doxygen-${PROJECT_NAME})
    set(CPACK_COMPONENT_EXAMPLES_DISPLAY_NAME "ROS2 System Handles doxygen")
    set(CPACK_COMPONENT_EXAMPLES_DESCRIPTION
        "eProsima ROS2 System Handles doxygen documentation")
    set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} doxygen-${PROJECT_NAME})
endif()
