/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 * Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <rcl/logging.h>

#include <is/core/Instance.hpp>
#include <is/utils/Convert.hpp>

#include <std_msgs/msg/string.hpp>

#include <yaml-cpp/yaml.h>
#include <chrono>

#include <gtest/gtest.h>

namespace is = eprosima::is;
namespace xtypes = eprosima::xtypes;

static is::utils::Logger logger("is::sh::ROS2_Dynamic::test::test_domain");

constexpr const size_t DOMAIN_ID_1 = 5;
constexpr const size_t DOMAIN_ID_2 = 10;

TEST(ROS2, Change_ROS2_Domain_id__ROS2_Galactic_or_older)
{
    char const* const argv[1] = {"is_ros2_test_domain_id"};
    if (!rclcpp::ok())
    {
        rclcpp::init(1, argv);
    }
    ASSERT_TRUE(rclcpp::ok());

    // Create the nodes in separate context, since in ROS2 Galactic each context uses
    // an unique DDS participant and thus creating the two nodes in the same context
    // would result in them having the same DOMAIN_ID.

    rclcpp::InitOptions init_options_1;
    if (rcl_logging_rosout_enabled())
    {
        init_options_1.auto_initialize_logging(false);
    }
    init_options_1.set_domain_id(DOMAIN_ID_1);

    const char* const argv_1[1] = {"is_ros2_test_domain_id_context_1"};
    auto context_1 = std::make_shared<rclcpp::Context>();
    context_1->init(1, argv_1, init_options_1);

    rclcpp::NodeOptions node_ops_1;
    node_ops_1.context(context_1);

    auto node_1 = std::make_shared<rclcpp::Node>("node_1", node_ops_1);

    rclcpp::InitOptions init_options_2;
    if (rcl_logging_rosout_enabled())
    {
        init_options_2.auto_initialize_logging(false);
    }
    init_options_2.set_domain_id(DOMAIN_ID_2);

    const char* const argv_2[1] = {"is_ros2_test_domain_id_context_2"};
    auto context_2 = std::make_shared<rclcpp::Context>();
    context_2->init(1, argv_2, init_options_2);

    rclcpp::NodeOptions node_ops_2;
    node_ops_2.context(context_2);

    auto node_2 = std::make_shared<rclcpp::Node>("node_2", node_ops_2);

    // Check correct DOMAIN ID for node_1 and node_2
    ASSERT_EQ(init_options_1.get_domain_id(), DOMAIN_ID_1);
    logger << is::utils::Logger::Level::INFO
           << "Domain ID for 'node_1': " << init_options_1.get_domain_id() << std::endl;

    ASSERT_EQ(init_options_2.get_domain_id(), DOMAIN_ID_2);
    logger << is::utils::Logger::Level::INFO
           << "Domain ID for 'node_2': " << init_options_2.get_domain_id() << std::endl;

    const std::string topic_name("string_topic");

#ifdef RCLCPP__QOS_HPP_
    const auto publisher =
            node_1->create_publisher<std_msgs::msg::String>(topic_name, rclcpp::SystemDefaultsQoS());
#else
    const auto publisher =
            node_1->create_publisher<std_msgs::msg::String>(topic_name);
#endif // ifdef RCLCPP__QOS_HPP_

    std::promise<std_msgs::msg::String> msg_promise;
    std::future<std_msgs::msg::String> msg_future = msg_promise.get_future();
    std::mutex node2_sub_mutex;
    auto node2_sub = [&](std_msgs::msg::String::UniquePtr msg)
            {
                std::unique_lock<std::mutex> lock(node2_sub_mutex);
                logger << is::utils::Logger::Level::DEBUG
                       << "Setting value to promise" << std::endl;
                msg_promise.set_value(*msg);
            };

#ifdef RCLCPP__QOS_HPP_
    const auto subscriber = node_2->create_subscription<std_msgs::msg::String>(
        topic_name, rclcpp::SystemDefaultsQoS(), node2_sub);
#else
    const auto subscriber = node_2->create_subscription<std_msgs::msg::String>(
        topic_name, node2_sub);
#endif // ifdef RCLCPP__QOS_HPP_

    std_msgs::msg::String pub_msg;
    pub_msg.set__data("Hello node");

    rclcpp::executors::SingleThreadedExecutor executor;
    using namespace std::chrono_literals;

    auto rclcpp_delay = 1s;
    publisher->publish(pub_msg);
    executor.spin_node_some(node_1);
    std::this_thread::sleep_for(rclcpp_delay);
    executor.spin_node_some(node_2);

    // In different domains the message should not be received
    ASSERT_NE(msg_future.wait_for(1s), std::future_status::ready);

    std::string yaml;

    yaml += "systems:\n";
    yaml += "   ros2_domain5: { type: ros2_dynamic, domain: 5, node_name: 'is_node_5', using: [std_msgs/String] } \n";
    yaml += "   ros2_domain10: { type: ros2_dynamic, domain: 10, node_name: 'is_node_10', using: [std_msgs/String]  } \n";
    yaml += "routes:\n";
    yaml += "   domain_5_to_10: { from: ros2_domain5, to: ros2_domain10 }\n";
    yaml += "topics:\n";
    yaml += "   string_topic: { type: 'std_msgs::msg::String', route: domain_5_to_10 }\n";

    std::cout << "YAML " << yaml << std::endl;

    YAML::Node config_node = YAML::Load(yaml);

    is::core::InstanceHandle handle = is::run_instance(
        config_node, { ROS2__ROSIDL__BUILD_DIR });

    ASSERT_TRUE(handle);

    // Wait for the Integration Service to start properly before publishing.
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < 30s)
    {
        executor.spin_node_some(node_1);
        std::this_thread::sleep_for(rclcpp_delay);
        executor.spin_node_some(node_2);
        if (msg_future.wait_for(100ms) == std::future_status::ready)
        {
            break;
        }

        publisher->publish(pub_msg);
    }

    ASSERT_EQ(msg_future.wait_for(0s), std::future_status::ready);

    std_msgs::msg::String received_msg = msg_future.get();

    ASSERT_EQ(pub_msg, received_msg);
}

int main(
        int argc,
        char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
