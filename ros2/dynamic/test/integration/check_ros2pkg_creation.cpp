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

#include <is/sh/mock/api.hpp>
#include <is/core/Instance.hpp>
#include <is/utils/Convert.hpp>
#include <is/utils/Log.hpp>

#include <is/sh/ros2/config.hpp>

#include <yaml-cpp/yaml.h>
#include <stdio.h>

#include <gtest/gtest.h>

#include <random>

namespace is = eprosima::is;
namespace xtypes = eprosima::xtypes;
using namespace std::chrono_literals;

using is::sh::ros2::ROS2_DISTRO;

static is::utils::Logger logger("is::sh::ROS2_Dynamic::test::check_ros2pkg_creation");

class ROS2Dynamic : public testing::Test
{
public:

    void SetUp() override
    {
        std::string yaml;

        yaml += "types:\n";
        yaml += "    idls:\n";
        yaml += "        - >\n";
        yaml += "            #include <std_msgs/msg/String.idl>\n\n";
        yaml += "            module custom_msgs\n";
        yaml += "            {\n";
        yaml += "                module msg\n";
        yaml += "                {\n";
        yaml += "                    struct Message\n";
        yaml += "                    {\n";
        yaml += "                        std_msgs::msg::String text;\n";
        yaml += "                    };\n";
        yaml += "                };\n";
        yaml += "            };\n";
        yaml += "    paths: [\"/opt/ros/";
        yaml += ROS2_DISTRO;
        yaml += "/share\"]\n";
        yaml += "systems:\n";
        yaml += "   ros2: { type: ros2_dynamic } \n";
        yaml += "   mock: { type: mock }\n";
        yaml += "routes:\n";
        yaml += "   mock_to_ros2: { from: mock, to: ros2 }\n";
        yaml += "   ros2_to_mock: { from: ros2, to: mock }\n";
        yaml += "topics:\n";
        yaml += "   transmit: { type: 'custom_msgs::msg::Message', route: ros2_to_mock }\n";
        yaml += "   echo: { type: 'custom_msgs::msg::Message', route: mock_to_ros2 }\n";

        logger << is::utils::Logger::Level::INFO << "YAML file:\n" << yaml << std::endl;

        YAML::Node config_node = YAML::Load(yaml);

        handle = std::make_unique<is::core::InstanceHandle>(is::run_instance(
                            config_node, {ROS2__ROSIDL__BUILD_DIR}));

        ASSERT_TRUE(handle.get());
        logger << is::utils::Logger::Level::INFO
               << "Integration Service launch finished" << std::endl;
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

    void test()
    {
        logger << is::utils::Logger::Level::INFO << "Execute ROS2_Dynamic test" << std::endl;

        std::ostringstream command;
        command << ". /opt/ros/" << ROS2_DISTRO << "/setup.sh && ros2 topic pub /test ";
        command << "custom_msgs/msg/Message \"{text: {data: 'thisisatest'}}\" --once" WAIT;

        FILE* pipe = popen(command.str().c_str(), "r");
        if (!pipe)
        {
            logger << is::utils::Logger::Level::ERROR
                   << "Failed to execute command: " << command.str() << std::endl;
        }
        ASSERT_NE(nullptr, pipe);

        char buffer[128];
        std::string output = "";
        while (!feof(pipe))
        {
            if (fgets(buffer, 128, pipe) != NULL)
            {
                output += buffer;
            }
        }

        logger << is::utils::Logger::Level::INFO << output << std::endl;
        int st = pclose(pipe);
        ASSERT_NE(0, WIFEXITED(st));
        ASSERT_EQ(0, WEXITSTATUS(st));

        auto res = output.find(
            "publishing #1: custom_msgs.msg.Message(text=std_msgs.msg.String(data='thisisatest'))");
        ASSERT_NE(std::string::npos, res);
    }

protected:

    std::unique_ptr<is::core::InstanceHandle> handle;
};

TEST_F(ROS2Dynamic, Check_ros2pkg_creation)
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
