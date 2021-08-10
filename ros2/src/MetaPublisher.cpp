/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "MetaPublisher.hpp"

#include <is/core/runtime/StringTemplate.hpp>

#include <is/sh/ros2/Factory.hpp>


namespace eprosima {
namespace is {
namespace sh {
namespace ros2 {

/**
 * @class MetaPublisher
 *        Create a TopicPublisher using the StringTemplate substitution paradigm.
 */
class MetaPublisher : public is::TopicPublisher
{
public:

    MetaPublisher(
            const core::StringTemplate&& topic_template,
            const eprosima::xtypes::DynamicType& message_type,
            rclcpp::Node& node,
            const rclcpp::QoS& qos_profile,
            const YAML::Node& /*unused*/)
        : _topic_template(std::move(topic_template))
        , _message_type(message_type)
        , _node(node)
        , _qos_profile(qos_profile)
        , logger_("is::sh::ROS2::Publisher")
    {
        // Do nothing
    }

    bool publish(
            const eprosima::xtypes::DynamicData& message) override final
    {
        const std::string topic_name = _topic_template.compute_string(message);

        const auto insertion = _publishers.insert(
            std::make_pair(std::move(topic_name), nullptr));
        const bool inserted = insertion.second;
        TopicPublisherPtr& publisher = insertion.first->second;

        if (inserted)
        {
            publisher = Factory::instance().create_publisher(
                _message_type, _node, topic_name, _qos_profile);
        }

        logger_ << utils::Logger::Level::INFO
            << "Sending message from Integration Service to ROS 2 for topic '" << topic_name
            << "': [[ " << message << " ]]" << std::endl;

        return publisher->publish(message);
    }

private:

    const core::StringTemplate _topic_template;
    const eprosima::xtypes::DynamicType& _message_type;
    rclcpp::Node& _node;
    const rclcpp::QoS _qos_profile;
    utils::Logger logger_;

    using TopicPublisherPtr = std::shared_ptr<TopicPublisher>;
    using PublisherMap = std::unordered_map<std::string, TopicPublisherPtr>;
    PublisherMap _publishers;

};

namespace {
//==============================================================================
std::string make_detail_string(
        const std::string& topic_name,
        const std::string& message_type)
{
    return
        "[Middleware: ROS2, topic template: "
        + topic_name + ", message type: " + message_type + "]";
}

} // anonymous namespace

//==============================================================================
std::shared_ptr<is::TopicPublisher> make_meta_publisher(
        const eprosima::xtypes::DynamicType& message_type,
        rclcpp::Node& node,
        const std::string& topic_name,
        const rclcpp::QoS& qos_profile,
        const YAML::Node& configuration)
{
    return std::make_shared<MetaPublisher>(
        core::StringTemplate(topic_name, make_detail_string(topic_name, message_type.name())),
        message_type, node, qos_profile, configuration);
}

} //  namespace ros2
} //  namespace sh
} //  namespace is
} //  namespace eprosima
