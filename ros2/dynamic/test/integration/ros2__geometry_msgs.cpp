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

#include <nav_msgs/srv/get_plan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

#include <random>
#include <sstream>

// TODO (@jamoralp): re-think or refactor these tests.

namespace is = eprosima::is;
namespace xtypes = eprosima::xtypes;

static is::utils::Logger logger("is::sh::ROS2_Dynamic::test::geometry_msgs");

const std::string print_pose(
        const geometry_msgs::msg::PoseStamped& ros2_pose)
{
    std::ostringstream oss;

    oss << "{ position: {"
        << " x: " << ros2_pose.pose.position.x << ","
        << " y: " << ros2_pose.pose.position.y << ","
        << " z: " << ros2_pose.pose.position.z
        << " }, orientation: { "
        << " w: " << ros2_pose.pose.orientation.w << ","
        << " x: " << ros2_pose.pose.orientation.x << ","
        << " y: " << ros2_pose.pose.orientation.y << ","
        << " z: " << ros2_pose.pose.orientation.z
        << " } }";

    return oss.str();
}

geometry_msgs::msg::PoseStamped generate_random_pose(
        const int sec = 0)
{
    std::mt19937 rng;
    // Use a fixed seed for deterministic test results
    rng.seed(39);
    std::uniform_real_distribution<double> dist(-100.0, 100.0);

    geometry_msgs::msg::PoseStamped ros2_pose;

    ros2_pose.pose.position.x = dist(rng);
    ros2_pose.pose.position.y = dist(rng);
    ros2_pose.pose.position.z = dist(rng);

    ros2_pose.pose.orientation.w = 1.0;
    ros2_pose.pose.orientation.x = 0.0;
    ros2_pose.pose.orientation.y = 0.0;
    ros2_pose.pose.orientation.z = 0.0;

    ros2_pose.header.frame_id = "map";
    ros2_pose.header.stamp.sec = sec;

    logger << is::utils::Logger::Level::DEBUG
           << "Generated random pose -> " << print_pose(ros2_pose) << std::endl;

    return ros2_pose;
}

void transform_pose_msg(
        const geometry_msgs::msg::PoseStamped& p,
        xtypes::WritableDynamicDataRef to)
{
    to["header"]["stamp"]["sec"] = p.header.stamp.sec;
    to["header"]["stamp"]["nanosec"] = p.header.stamp.nanosec;
    to["header"]["frame_id"] = "map";
    to["pose"]["position"]["x"] = p.pose.position.x;
    to["pose"]["position"]["y"] = p.pose.position.y;
    to["pose"]["position"]["z"] = p.pose.position.z;
    to["pose"]["orientation"]["x"] = p.pose.orientation.x;
    to["pose"]["orientation"]["y"] = p.pose.orientation.y;
    to["pose"]["orientation"]["z"] = p.pose.orientation.z;
    to["pose"]["orientation"]["w"] = p.pose.orientation.w;
}

xtypes::DynamicData generate_plan_request_msg(
        const xtypes::DynamicType& request_type,
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const float tolerance = 1e-3f)
{
    xtypes::DynamicData message(request_type);

    transform_pose_msg(goal, message["goal"]);
    transform_pose_msg(start, message["start"]);
    message["tolerance"] = tolerance;

    return message;
}

const std::string print_header(
        const std_msgs::msg::Header& header)
{
    std::ostringstream oss;

    oss << "[stamp: " << header.stamp.sec << " | " << header.stamp.nanosec
        << "] [frame_id: " << header.stamp.sec + "]";

    return oss.str();
}

void compare_plans(
        const nav_msgs::srv::GetPlan_Response::_plan_type& plan_a,
        const nav_msgs::srv::GetPlan_Response::_plan_type& plan_b)
{
    const bool header_matches = (plan_a.header == plan_b.header);
    EXPECT_TRUE(header_matches);

    if (!header_matches)
    {
        logger << is::utils::Logger::Level::WARN
               << "[compare_plans] Headers did not match:\n\t -- Header A: "
               << print_header(plan_a.header) << "\n\t -- Header B: "
               << print_header(plan_b.header) << std::endl;
    }
    else
    {
        logger << is::utils::Logger::Level::DEBUG
               << "[compare_plans] Headers A and B matched: "
               << print_header(plan_a.header) << std::endl;
    }

    ASSERT_EQ(plan_a.poses.size(), plan_b.poses.size());
    for (std::size_t i = 0; i < plan_a.poses.size(); ++i)
    {
        const bool pose_matches = (plan_a.poses[i] == plan_b.poses[i]);
        EXPECT_TRUE(pose_matches);

        if (!pose_matches)
        {
            logger << is::utils::Logger::Level::WARN
                   << "[compare_plans] Poses at index [" << i << "]"
                   << " did not match. Pose A is: " << print_pose(plan_a.poses[i])
                   << ", and pose B is: " << print_pose(plan_b.poses[i]) << std::endl;
        }
        else
        {
            logger << is::utils::Logger::Level::DEBUG
                   << "[compare_plans] Poses A and B matched: "
                   << print_pose(plan_a.poses[i]) << std::endl;
        }
    }
}

TEST(ROS2Dynamic, Publish_subscribe_between_ros2_and_mock)
{
    using namespace std::chrono_literals;

    const double tolerance = 1e-8;

    std::string yaml;

    yaml += "systems:\n";
    yaml += "   ros2: { type: ros2_dynamic, using: [geometry_msgs/Pose] } \n";
    yaml += "   mock: { type: mock, types-from: ros2 }\n";
    yaml += "routes:\n";
    yaml += "   mock_to_ros2: { from: mock, to: ros2 }\n";
    yaml += "   ros2_to_mock: { from: ros2, to: mock }\n";
    yaml += "topics:\n";
    yaml += "   transmit_pose: { type: 'geometry_msgs::msg::Pose', route: ros2_to_mock }\n";
    yaml += "   echo_pose: { type: 'geometry_msgs::msg::Pose', route: mock_to_ros2 }\n";

    std::cout << "YAML " << yaml << std::endl;

    YAML::Node config_node = YAML::Load(yaml);

    is::core::InstanceHandle handle = is::run_instance(
        config_node, {ROS2__ROSIDL__BUILD_DIR});

    ASSERT_TRUE(handle);

    char const* const argv[1] = {"is_ros2_test_geometry_msgs"};
    rclcpp::init(1, argv);

    rclcpp::Node::SharedPtr ros2 = std::make_shared<rclcpp::Node>("ros2_test");
    rclcpp::executors::SingleThreadedExecutor executor;

    ASSERT_TRUE( rclcpp::ok() );

    executor.add_node(ros2);

    const auto publisher =
#ifndef RCLCPP__QOS_HPP_
            ros2->create_publisher<geometry_msgs::msg::Pose>("transmit_pose");
#else
            ros2->create_publisher<geometry_msgs::msg::Pose>(
        "transmit_pose", rclcpp::SystemDefaultsQoS());
#endif // RCLCPP__QOS_HPP_
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

                mock_sub_value_received = true;
                msg_promise.set_value(msg);
            };
    ASSERT_TRUE(is::sh::mock::subscribe("transmit_pose", mock_sub));

    geometry_msgs::msg::Pose ros2_pose = generate_random_pose().pose;

    publisher->publish(ros2_pose);

    executor.spin_some();

    // Wait no longer than a few seconds for the message to arrive. If it's not
    // ready by that time, then something is probably broken with the test, and
    // we should quit instead of waiting for the future and potentially hanging
    // forever.
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < 1min)
    {
        executor.spin_some();
        if (msg_future.wait_for(1s) == std::future_status::ready)
        {
            break;
        }

        publisher->publish(ros2_pose);
    }

    ASSERT_EQ(msg_future.wait_for(0s), std::future_status::ready);
    xtypes::DynamicData received_msg = msg_future.get();

    EXPECT_EQ(received_msg.type().name(), "geometry_msgs::msg::Pose");

    xtypes::ReadableDynamicDataRef position = received_msg["position"];
    xtypes::ReadableDynamicDataRef orientation = received_msg["orientation"];

    #define TEST_POSITION_OF( u ) \
    { \
        const double u = position[#u]; \
        ASSERT_NEAR(u, ros2_pose.position.u, tolerance); \
    }

    TEST_POSITION_OF(x);
    TEST_POSITION_OF(y);
    TEST_POSITION_OF(z);

    bool promise_sent = false;
    std::promise<geometry_msgs::msg::Pose> pose_promise;
    auto pose_future = pose_promise.get_future();
    std::mutex echo_mutex;
    auto echo_sub = [&](geometry_msgs::msg::Pose::UniquePtr msg)
            {
                std::unique_lock<std::mutex> lock(echo_mutex);

                // promises will throw an exception if set_value(~) is called more than
                // once, so we'll guard against that.
                if (promise_sent)
                {
                    return;
                }

                promise_sent = true;
                pose_promise.set_value(*msg);
            };

#ifndef RCLCPP__QOS_HPP_
    const auto subscriber = ros2->create_subscription<geometry_msgs::msg::Pose>(
        "echo_pose", echo_sub);
#else
    const auto subscriber = ros2->create_subscription<geometry_msgs::msg::Pose>(
        "echo_pose", rclcpp::SystemDefaultsQoS(), echo_sub);
#endif // RCLCPP__QOS_HPP_
    ASSERT_TRUE(subscriber);

    // Keep spinning and publishing while we wait for the promise to be
    // delivered. Try to cycle this for no more than a few seconds. If it's not
    // finished by that time, then something is probably broken with the test or
    // with Integratoion Service, and we should quit instead of waiting for the future and
    // potentially hanging forever.
    start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < 1min)
    {
        executor.spin_some();

        is::sh::mock::publish_message("echo_pose", received_msg);

        executor.spin_some();
        if (pose_future.wait_for(1s) == std::future_status::ready)
        {
            break;
        }
    }

    ASSERT_EQ(pose_future.wait_for(0s), std::future_status::ready);
    geometry_msgs::msg::Pose received_pose = pose_future.get();

    EXPECT_EQ(ros2_pose, received_pose);

    // Destroy ros2 instance node
    executor.remove_node(ros2);
    ros2.reset();

    // Quit and wait for no more than a minute. We don't want the test to get
    // hung here indefinitely in the case of an error.
    handle.quit().wait_for(1min);

    // Require that it's no longer running. If it is still running, then it is
    // probably stuck, and we should forcefully quit.
    ASSERT_TRUE(!handle.running());
    ASSERT_TRUE(handle.wait() == 0);

    rclcpp::shutdown();
}

int main(
        int argc,
        char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
