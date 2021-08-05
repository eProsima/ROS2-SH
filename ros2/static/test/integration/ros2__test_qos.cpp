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

#include <is/sh/mock/api.hpp>

#include <std_msgs/msg/string.hpp>

#include <yaml-cpp/yaml.h>
#include <chrono>

#include <gtest/gtest.h>

namespace is = eprosima::is;
namespace xtypes = eprosima::xtypes;
using namespace std::chrono_literals;

static is::utils::Logger logger("is::sh::ROS2::test::test_qos");

class ROS2QoS : public ::testing::Test
{
public:

    void set_up(
        const std::string& yaml)
    {
        std::cout << "YAML " << yaml << std::endl;

        YAML::Node config_node = YAML::Load(yaml);

        handle = std::make_unique<is::core::InstanceHandle>(is::run_instance(
            config_node, {ROS2__ROSIDL__BUILD_DIR}));

        ASSERT_TRUE(handle.get());

        ros2 = std::make_shared<rclcpp::Node>("ros2_test");
        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        ASSERT_TRUE( rclcpp::ok() );

        executor->add_node(ros2);
    }

    void test(
        const rclcpp::QoS& qos_profile_publisher,
        bool compatible)
    {
        const auto publisher =
                    ros2->create_publisher<std_msgs::msg::String>("transmit", qos_profile_publisher);

        ASSERT_TRUE(publisher);

        std::promise<xtypes::DynamicData> msg_promise;
        std::future<xtypes::DynamicData> msg_future = msg_promise.get_future();
        std::mutex mock_sub_mutex;
        bool mock_sub_value_received = false;
        auto mock_sub = [&](const xtypes::DynamicData& msg)
            {
                std::unique_lock<std::mutex> lock(mock_sub_mutex);
                if (mock_sub_value_received)
                {
                    return;
                }

                logger << is::utils::Logger::Level::INFO
                    << "Message received" << std::endl;

                mock_sub_value_received = true;
                msg_promise.set_value(msg);
            };

        ASSERT_TRUE(is::sh::mock::subscribe("transmit", mock_sub));

        std_msgs::msg::String message;
        message.set__data("Transmiting");

        publisher->publish(message);

        executor->spin_some();

        // Try sending several times to assure that the message is not received
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < 10s)
        {
            executor->spin_some();
            if (msg_future.wait_for(100ms) == std::future_status::ready)
            {
                break;
            }

            publisher->publish(message);
        }

        if (!compatible)
        {
            ASSERT_NE(msg_future.wait_for(0s), std::future_status::ready);
        }
        else
        {
            ASSERT_EQ(msg_future.wait_for(0s), std::future_status::ready);
            xtypes::DynamicData received_msg = msg_future.get();

            EXPECT_EQ(received_msg.type().name(), "std_msgs/String");

            xtypes::ReadableDynamicDataRef xtypes_msg = received_msg;
            typename std_msgs::msg::String::_data_type ros2_field;
            is::utils::Convert<typename std_msgs::msg::String::_data_type>::from_xtype_field(xtypes_msg["data"], ros2_field);
            EXPECT_EQ(ros2_field, message.data);
        }

        // Destroy ros2 instance node
        executor->remove_node(ros2);
        ros2.reset();

        // Quit and wait for no more than a minute. We don't want the test to get
        // hung here indefinitely in the case of an error.
        handle->quit().wait_for(1min);

        // Require that it's no longer running. If it is still running, then it is
        // probably stuck, and we should forcefully quit.
        ASSERT_TRUE(!handle->running());
        ASSERT_TRUE(handle->wait() == 0);
    }

protected:

    std::shared_ptr<rclcpp::Node> ros2;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::unique_ptr<is::core::InstanceHandle> handle;

};

TEST_F(ROS2QoS, Durability_Incompatible_QoS)
{
    std::string yaml;

    yaml += "systems:\n";
    yaml += "   ros2: { type: ros2 } \n";
    yaml += "   mock: { type: mock, types-from: ros2 }\n";
    yaml += "routes:\n";
    yaml += "   mock_to_ros2: { from: mock, to: ros2 }\n";
    yaml += "   ros2_to_mock: { from: ros2, to: mock }\n";
    yaml += "topics:\n";
    yaml += "   transmit: { type: 'std_msgs/String', route: ros2_to_mock, ";
    yaml += "   ros2: { qos: { durability: TRANSIENT_LOCAL } } }\n";

    set_up(yaml);

    rclcpp::QoS qos_profile_publisher(10);
    qos_profile_publisher.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    test(qos_profile_publisher, false);

}

TEST_F(ROS2QoS, Durability_Compatible_QoS)
{
    std::string yaml;

    yaml += "systems:\n";
    yaml += "   ros2: { type: ros2 } \n";
    yaml += "   mock: { type: mock, types-from: ros2 }\n";
    yaml += "routes:\n";
    yaml += "   mock_to_ros2: { from: mock, to: ros2 }\n";
    yaml += "   ros2_to_mock: { from: ros2, to: mock }\n";
    yaml += "topics:\n";
    yaml += "   transmit: { type: 'std_msgs/String', route: ros2_to_mock, ";
    yaml += "   ros2: { qos: { durability: VOLATILE } } }\n";

    set_up(yaml);

    rclcpp::QoS qos_profile_publisher(10);
    qos_profile_publisher.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    test(qos_profile_publisher, true);

}

TEST_F(ROS2QoS, Deadline_Incompatible_QoS)
{
    std::string yaml;

    yaml += "systems:\n";
    yaml += "   ros2: { type: ros2 } \n";
    yaml += "   mock: { type: mock, types-from: ros2 }\n";
    yaml += "routes:\n";
    yaml += "   mock_to_ros2: { from: mock, to: ros2 }\n";
    yaml += "   ros2_to_mock: { from: ros2, to: mock }\n";
    yaml += "topics:\n";
    yaml += "   transmit: { type: 'std_msgs/String', route: ros2_to_mock, ";
    yaml += "   ros2: { qos: { deadline: { sec: 1 } } } }\n";

    set_up(yaml);

    rclcpp::QoS qos_profile_publisher(10);
    qos_profile_publisher.deadline(std::chrono::milliseconds(2000));

    test(qos_profile_publisher, false);

}

TEST_F(ROS2QoS, Deadline_Compatible_QoS)
{
    std::string yaml;

    yaml += "systems:\n";
    yaml += "   ros2: { type: ros2 } \n";
    yaml += "   mock: { type: mock, types-from: ros2 }\n";
    yaml += "routes:\n";
    yaml += "   mock_to_ros2: { from: mock, to: ros2 }\n";
    yaml += "   ros2_to_mock: { from: ros2, to: mock }\n";
    yaml += "topics:\n";
    yaml += "   transmit: { type: 'std_msgs/String', route: ros2_to_mock, ";
    yaml += "   ros2: { qos: { deadline: { sec: 2 } } } }\n";

    set_up(yaml);

    rclcpp::QoS qos_profile_publisher(10);
    qos_profile_publisher.deadline(std::chrono::milliseconds(2000));

    test(qos_profile_publisher, true);

}

TEST_F(ROS2QoS, Liveliness_Incompatible_QoS)
{
    std::string yaml;

    yaml += "systems:\n";
    yaml += "   ros2: { type: ros2 } \n";
    yaml += "   mock: { type: mock, types-from: ros2 }\n";
    yaml += "routes:\n";
    yaml += "   mock_to_ros2: { from: mock, to: ros2 }\n";
    yaml += "   ros2_to_mock: { from: ros2, to: mock }\n";
    yaml += "topics:\n";
    yaml += "   transmit: { type: 'std_msgs/String', route: ros2_to_mock, ";
    yaml += "   ros2: { qos: { liveliness: { kind: MANUAL_BY_TOPIC } } } }\n";

    set_up(yaml);

    rclcpp::QoS qos_profile_publisher(10);
    qos_profile_publisher.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);

    test(qos_profile_publisher, false);

}

TEST_F(ROS2QoS, Liveliness_Compatible_QoS)
{
    std::string yaml;

    yaml += "systems:\n";
    yaml += "   ros2: { type: ros2 } \n";
    yaml += "   mock: { type: mock, types-from: ros2 }\n";
    yaml += "routes:\n";
    yaml += "   mock_to_ros2: { from: mock, to: ros2 }\n";
    yaml += "   ros2_to_mock: { from: ros2, to: mock }\n";
    yaml += "topics:\n";
    yaml += "   transmit: { type: 'std_msgs/String', route: ros2_to_mock, ";
    yaml += "   ros2: { qos: { liveliness: { kind: AUTOMATIC } } } }\n";

    set_up(yaml);

    rclcpp::QoS qos_profile_publisher(10);
    qos_profile_publisher.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);

    test(qos_profile_publisher, true);

}

TEST_F(ROS2QoS, Lease_Duration_Incompatible_QoS)
{
    std::string yaml;

    yaml += "systems:\n";
    yaml += "   ros2: { type: ros2 } \n";
    yaml += "   mock: { type: mock, types-from: ros2 }\n";
    yaml += "routes:\n";
    yaml += "   mock_to_ros2: { from: mock, to: ros2 }\n";
    yaml += "   ros2_to_mock: { from: ros2, to: mock }\n";
    yaml += "topics:\n";
    yaml += "   transmit: { type: 'std_msgs/String', route: ros2_to_mock, ";
    yaml += "   ros2: { qos: { liveliness: { sec: 1 } } } }\n";

    set_up(yaml);

    rclcpp::QoS qos_profile_publisher(10);
    qos_profile_publisher.liveliness_lease_duration(std::chrono::milliseconds(2000));

    test(qos_profile_publisher, false);
}

TEST_F(ROS2QoS, Lease_Duration_Compatible_QoS)
{
    std::string yaml;

    yaml += "systems:\n";
    yaml += "   ros2: { type: ros2 } \n";
    yaml += "   mock: { type: mock, types-from: ros2 }\n";
    yaml += "routes:\n";
    yaml += "   mock_to_ros2: { from: mock, to: ros2 }\n";
    yaml += "   ros2_to_mock: { from: ros2, to: mock }\n";
    yaml += "topics:\n";
    yaml += "   transmit: { type: 'std_msgs/String', route: ros2_to_mock, ";
    yaml += "   ros2: { qos: { liveliness: { sec: 3 } } } }\n";

    set_up(yaml);

    rclcpp::QoS qos_profile_publisher(10);
    qos_profile_publisher.liveliness_lease_duration(std::chrono::milliseconds(2000));

    test(qos_profile_publisher, true);

}

TEST_F(ROS2QoS, Reliability_Incompatible_QoS)
{
    std::string yaml;

    yaml += "systems:\n";
    yaml += "   ros2: { type: ros2 } \n";
    yaml += "   mock: { type: mock, types-from: ros2 }\n";
    yaml += "routes:\n";
    yaml += "   mock_to_ros2: { from: mock, to: ros2 }\n";
    yaml += "   ros2_to_mock: { from: ros2, to: mock }\n";
    yaml += "topics:\n";
    yaml += "   transmit: { type: 'std_msgs/String', route: ros2_to_mock, ";
    yaml += "   ros2: { qos: { reliability: RELIABLE } } }\n";

    set_up(yaml);

    rclcpp::QoS qos_profile_publisher(10);
    qos_profile_publisher.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    test(qos_profile_publisher, false);

}

TEST_F(ROS2QoS, Reliability_Compatible_QoS)
{
    std::string yaml;

    yaml += "systems:\n";
    yaml += "   ros2: { type: ros2 } \n";
    yaml += "   mock: { type: mock, types-from: ros2 }\n";
    yaml += "routes:\n";
    yaml += "   mock_to_ros2: { from: mock, to: ros2 }\n";
    yaml += "   ros2_to_mock: { from: ros2, to: mock }\n";
    yaml += "topics:\n";
    yaml += "   transmit: { type: 'std_msgs/String', route: ros2_to_mock, ";
    yaml += "   ros2: { qos: { reliability: BEST_EFFORT } } }\n";

    set_up(yaml);

    rclcpp::QoS qos_profile_publisher(10);
    qos_profile_publisher.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    test(qos_profile_publisher, true);
}

int main(
        int argc,
        char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
