// generated from is-ros2/resources/convert_msg.cpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating is/rosidl/ros2/<package>/src/msg/convert__msg__<msg>.cpp files
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Either 'msg' or 'srv'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################

@{
camelcase_msg_type = spec.base_type.type
underscore_msg_type = get_header_filename_from_msg_name(camelcase_msg_type)

namespace_parts = [
    'convert', spec.base_type.pkg_name, 'msg', underscore_msg_type]
namespace_variable = '__'.join(namespace_parts)

conversion_dependency = 'is/rosidl/ros2/{}/msg/convert__msg__{}.hpp'.format(
    spec.base_type.pkg_name, camelcase_msg_type)
}@

// Include the API header for this message type
#include <@(conversion_dependency)>
// Include the Factory header so we can add this message type to the Factory
#include <is/sh/ros2/Factory.hpp>

// Include the Node API so we can subscribe and advertise
#include <rclcpp/node.hpp>

// TODO(jamoralp): Add utils::Logger traces here
namespace eprosima {
namespace is {
namespace sh {
namespace ros2 {
namespace @(namespace_variable) {

//==============================================================================
namespace {
TypeToFactoryRegistrar register_type(g_msg_name, &type);
} // anonymous namespace

//==============================================================================
class Subscription final
{
public:

    Subscription(
            rclcpp::Node& node,
            TopicSubscriberSystem::SubscriptionCallback* callback,
            const std::string& topic_name,
            const xtypes::DynamicType& message_type,
            const rclcpp::QoS& qos_profile)
        : _callback(callback)
        , _message_type(message_type)
        , _topic_name(topic_name)
    {
        auto subscription_options = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
        subscription_options.ignore_local_publications = true; // Enable ignore_local_publications option
        _subscription = node.create_subscription<Ros2_Msg>(
            topic_name,
            qos_profile,
            [=](Ros2_Msg::UniquePtr msg)
            {
                this->subscription_callback(*msg);
            },
            subscription_options);
    }

private:

    void subscription_callback(
            const Ros2_Msg& msg)
    {
        logger << utils::Logger::Level::INFO
               << "Receiving message from ROS 2 for topic '"
               << _topic_name << "'" << std::endl;

        xtypes::DynamicData data(_message_type);
        convert_to_xtype(msg, data);

        logger << utils::Logger::Level::INFO
                << "Received message: [[ " << data << " ]]" << std::endl;

        (*_callback)(data, nullptr);
    }

    // Save the callback that we were given by the is-ros2 plugin
    TopicSubscriberSystem::SubscriptionCallback* _callback;

    const xtypes::DynamicType& _message_type;

    std::string _topic_name;

    // Hang onto the subscription handle to make sure the connection to the topic
    // stays alive
    rclcpp::Subscription<Ros2_Msg>::SharedPtr _subscription;

};

//==============================================================================
std::shared_ptr<void> subscribe(
        rclcpp::Node& node,
        const std::string& topic_name,
        const xtypes::DynamicType& message_type,
        TopicSubscriberSystem::SubscriptionCallback* callback,
        const rclcpp::QoS& qos_profile)
{
    return std::make_shared<Subscription>(
        node, callback, topic_name, message_type, qos_profile);
}

namespace {
SubscriptionToFactoryRegistrar register_subscriber(g_msg_name, &subscribe);
} // anonymous namespace

//==============================================================================
class Publisher final : public virtual is::TopicPublisher
{
public:

    Publisher(
            rclcpp::Node& node,
            const std::string& topic_name,
            const rclcpp::QoS& qos_profile)
        : _topic_name(topic_name)
    {
        _publisher = node.create_publisher<Ros2_Msg>(
            topic_name,
            qos_profile);
    }

    bool publish(
            const xtypes::DynamicData& message) override
    {
        Ros2_Msg ros2_msg;
        convert_to_ros2(message, ros2_msg);

        logger << utils::Logger::Level::INFO
            << "Sending message from Integration Service to ROS 2 for topic '" << _topic_name << "': "
            << "[[ " << message << " ]]" << std::endl;

        _publisher->publish(ros2_msg);
        return true;
    }

private:

    rclcpp::Publisher<Ros2_Msg>::SharedPtr _publisher;

    std::string _topic_name;

};

//==============================================================================
std::shared_ptr<is::TopicPublisher> make_publisher(
        rclcpp::Node& node,
        const std::string& topic_name,
        const rclcpp::QoS& qos_profile)
{
    return std::make_shared<Publisher>(node, topic_name, qos_profile);
}

namespace {
PublisherToFactoryRegistrar register_publisher(g_msg_name, &make_publisher);
} // namespace {

} //  namespace @(namespace_variable)
} //  namespace ros2
} //  namespace sh
} //  namespace is
} //  namespace eprosima
