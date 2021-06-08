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
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <is/sh/mock/api.hpp>
#include <is/core/Instance.hpp>
#include <is/utils/Convert.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>

#include <yaml-cpp/yaml.h>
#include <cxxabi.h>

#include <gtest/gtest.h>

#include <random>

namespace is = eprosima::is;
namespace xtypes = eprosima::xtypes;
using namespace std::chrono_literals;

static is::utils::Logger logger("is::sh::ROS2::test::primitive_msgs");

template<typename T>
std::string get_type_name(T type)
{
    // Get name of the templated type
    int status;
    std::string tname = typeid(T).name();
    char *demangled_name = abi::__cxa_demangle(tname.c_str(), NULL, NULL, &status);
    if (status == 0)
    {
        tname = demangled_name;
    }
    std::size_t found = tname.find("_<");
    if (found != std::string::npos)
    {
        tname = tname.substr(0, found);
        found = tname.find("::msg");
        if (found != std::string::npos)
        {
            tname = tname.replace(found, 5, "");
        }
        size_t index = 0;
        while (true)
        {
            index = tname.find("::", index);
            if (index == std::string::npos)
            {
                break;
            }
            tname.replace(index, 2, "/");
            index += 2;
        }
    }
    return tname;
}

template<typename T>
class ROS2 : public testing::Test
{
public:

    std::shared_ptr<rclcpp::Node> ros2;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::unique_ptr<is::core::InstanceHandle> handle;

    std::promise<xtypes::DynamicData> msg_promise;
    std::future<xtypes::DynamicData> msg_future = msg_promise.get_future();
    bool is_msg_received = false;
    std::mutex mock_sub_mutex;

    bool promise_sent = false;
    std::promise<T> t_promise;
    std::future<T> t_future = t_promise.get_future();
    std::mutex echo_sub_mutex;

    void SetUp() override
    {
        std::string yaml;

        yaml += "systems:\n";
        yaml += "   ros2: { type: ros2 } \n";
        yaml += "   mock: { type: mock, types-from: ros2 }\n";
        yaml += "routes:\n";
        yaml += "   mock_to_ros2: { from: mock, to: ros2 }\n";
        yaml += "   ros2_to_mock: { from: ros2, to: mock }\n";
        yaml += "topics:\n";
        yaml += "   transmit: { type: '";
        yaml += get_type_name<T>(T());
        yaml += "', route: ros2_to_mock }\n";
        yaml += "   echo: { type: '";
        yaml += get_type_name<T>(T());
        yaml += "', route: mock_to_ros2 }\n";

        std::cout << "YAML " << yaml << std::endl;

        YAML::Node config_node = YAML::Load(yaml);

        handle = std::make_unique<is::core::InstanceHandle>(is::run_instance(
            config_node, {ROS2__ROSIDL__BUILD_DIR}));

        ASSERT_TRUE(handle.get());

        ros2 = std::make_shared<rclcpp::Node>("ros2_test");

        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        ASSERT_TRUE( rclcpp::ok() );
    }

    void test()
    {
        const auto publisher =
    #ifndef RCLCPP__QOS_HPP_
                ros2->create_publisher<T>("transmit");
    #else
                ros2->create_publisher<T>("transmit", rclcpp::SystemDefaultsQoS());
    #endif // RCLCPP__QOS_HPP_
        ASSERT_TRUE(publisher);

        auto mock_sub = [&](const xtypes::DynamicData& msg)
                {
                    std::unique_lock<std::mutex> lock(mock_sub_mutex);
                    if (is_msg_received)
                    {
                        return;
                    }

                    is_msg_received = true;
                    msg_promise.set_value(msg);
                };

        ASSERT_TRUE(is::sh::mock::subscribe("transmit", mock_sub));

        T ros2_msg = generate_random_primitive(455);

        publisher->publish(ros2_msg);

        executor->spin_node_some(ros2);

        // Keep spinning while we wait for the promise to be delivered. Try cycle
        // this for no more than a few seconds. If it's not finished by that time,
        // then something is probably broken with the test or with the Integration Service, and we
        // should quit instead of waiting for the future and potentially hanging
        // forever.
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < 30s)
        {
            executor->spin_node_some(ros2);
            if (msg_future.wait_for(100ms) == std::future_status::ready)
            {
                break;
            }

            publisher->publish(ros2_msg);
        }

        ASSERT_EQ(msg_future.wait_for(0s), std::future_status::ready);
        xtypes::DynamicData received_msg = msg_future.get();

        EXPECT_EQ(received_msg.type().name(), get_type_name<T>(T()));

        xtypes::ReadableDynamicDataRef xtypes_primitive = received_msg;
        typename T::_data_type ros2_field;
        is::utils::Convert<typename T::_data_type>::from_xtype_field( xtypes_primitive["data"], ros2_field);
        EXPECT_EQ(ros2_field, ros2_msg.data);

        auto echo_sub = [&](typename T::UniquePtr msg)
                {
                    std::unique_lock<std::mutex> lock(echo_sub_mutex);
                    // promises will throw an exception if set_value(~) is called more than
                    // once, so we'll guard against that.
                    if (promise_sent)
                    {
                        return;
                    }

                    promise_sent = true;
                    t_promise.set_value(*msg);
                };

        const auto subscriber =
    #ifndef RCLCPP__QOS_HPP_
                ros2->create_subscription<T>("echo", echo_sub);
    #else
                ros2->create_subscription<T>("echo", rclcpp::SystemDefaultsQoS(), echo_sub);
    #endif // RCLCPP__QOS_HPP_
        ASSERT_TRUE(subscriber);

        // Keep spinning and publishing while we wait for the promise to be
        // delivered. Try cycle this for no more than a few seconds. If it's not
        // finished by that time, then something is probably broken with the test or
        // with the Integration Service, and we should quit instead of waiting for the future and
        // potentially hanging forever.
        start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < 30s)
        {
            executor->spin_node_some(ros2);

            is::sh::mock::publish_message("echo", received_msg);

            executor->spin_node_some(ros2);
            if (t_future.wait_for(100ms) == std::future_status::ready)
            {
                break;
            }
        }

        ASSERT_EQ(t_future.wait_for(0s), std::future_status::ready);
        T received_t = t_future.get();

        // Check that the msgs match
        EXPECT_EQ(received_t.data, ros2_msg.data);
    }

    void TearDown() override
    {
        // Quit and wait for no more than a minute. We don't want the test to get
        // hung here indefinitely in the case of an error.
        handle->quit().wait_for(1min);

        // Require that it's no longer running. If it is still running, then it is
        // probably stuck, and we should forcefully quit.
        ASSERT_TRUE(!handle->running());
        ASSERT_EQ(handle->wait(), 0);
    }

    T generate_random_primitive(
            const std::size_t seed)
    {
        std::mt19937 rng;
        // Use consistent seeds for deterministic test results
        rng.seed(64 + seed);

        T primitive;

        if (std::is_same<T, std_msgs::msg::Bool>::value)
        {
            primitive.data = std::uniform_int_distribution<uint8_t>(0, 2)(rng);
        }
        else if (std::is_same<T, std_msgs::msg::Byte>::value)
        {
            primitive.data = std::uniform_int_distribution<uint8_t>()(rng);
        }
        else if (std::is_same<T, std_msgs::msg::Char>::value)
        {
            primitive.data = std::uniform_int_distribution<uint8_t>()(rng);
        }
        else if (std::is_same<T, std_msgs::msg::Int8>::value)
        {
            primitive.data = std::uniform_int_distribution<int8_t>()(rng);
        }
        else if (std::is_same<T, std_msgs::msg::Int16>::value)
        {
            primitive.data = std::uniform_int_distribution<int16_t>()(rng);
        }
        else if (std::is_same<T, std_msgs::msg::Int32>::value)
        {
            primitive.data = std::uniform_int_distribution<int32_t>()(rng);
        }
        else if (std::is_same<T, std_msgs::msg::Int64>::value)
        {
            primitive.data = std::uniform_int_distribution<int64_t>()(rng);
        }
        else if (std::is_same<T, std_msgs::msg::UInt8>::value)
        {
            primitive.data = std::uniform_int_distribution<uint8_t>()(rng);
        }
        else if (std::is_same<T, std_msgs::msg::UInt16>::value)
        {
            primitive.data = std::uniform_int_distribution<uint16_t>()(rng);
        }
        else if (std::is_same<T, std_msgs::msg::UInt32>::value)
        {
            primitive.data = std::uniform_int_distribution<uint32_t>()(rng);
        }
        else if (std::is_same<T, std_msgs::msg::UInt64>::value)
        {
            primitive.data = std::uniform_int_distribution<uint64_t>()(rng);
        }
        else if (std::is_same<T, std_msgs::msg::Float32>::value)
        {
            primitive.data = std::uniform_real_distribution<float>()(rng);
        }
        else if (std::is_same<T, std_msgs::msg::Float64>::value)
        {
            primitive.data = std::uniform_real_distribution<double>()(rng);
        }

        return primitive;
    }
};

using ROS2PrimitiveTypes = testing::Types<std_msgs::msg::Bool, std_msgs::msg::Byte, std_msgs::msg::Char,
    std_msgs::msg::Int8, std_msgs::msg::Int16, std_msgs::msg::Int32, std_msgs::msg::Int64, std_msgs::msg::UInt8,
    std_msgs::msg::UInt16, std_msgs::msg::UInt32, std_msgs::msg::UInt64, std_msgs::msg::Float32,
    std_msgs::msg::Float64>;
TYPED_TEST_CASE(ROS2, ROS2PrimitiveTypes);

TYPED_TEST(ROS2, Transmit_and_receive_primitive_messages)
{
    this->test();
}

int main(
        int argc,
        char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
