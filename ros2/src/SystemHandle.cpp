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

#include "SystemHandle.hpp"
#include "MetaPublisher.hpp"

#include <is/sh/ros2/Factory.hpp>

#include <is/core/runtime/MiddlewareInterfaceExtension.hpp>
#include <is/core/runtime/Search.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rcl/logging.h>

#include <thread>

namespace eprosima {
namespace is {
namespace sh {
namespace ros2 {

namespace {

bool set_platform_env(
        const std::string& variable,
        const std::string& value,
        const bool overwrite)
{
#ifdef WIN32
    std::ostringstream aux_d;
    aux_d << variable << "=" << value;
    return (0 == _putenv(aux_d.str().c_str()));
#else
    return (0 == setenv(variable.c_str(), value.c_str(), overwrite));
#endif // WIN32
}

int unset_platform_env(
        const std::string& variable)
{
#ifdef WIN32
    return set_platform_env(variable, "", false);
#else
    return (0 == unsetenv(variable.c_str()));
#endif // WIN32
}

rmw_qos_profile_t parse_rmw_qos_configuration(
        const YAML::Node& /*configuration*/)
{
    // TODO(MXG): Parse the configuration node
    // This is currently considered low-priority
    return rmw_qos_profile_default;
}

} //  anonymous namespace

//==============================================================================
void SystemHandle::print_missing_mix_file(
        const std::string& msg_or_srv,
        const std::string& type,
        const std::vector<std::string>& checked_paths)
{
    _logger << utils::Logger::Level::ERROR
            << "Could not find .mix file for " << msg_or_srv << " type: '"
            << type << "'.\n -- Make sure that you have generated "
            << "the 'is-ros2' extension for that message type by calling "
            << "'is_ros2_rosidl_mix(PACKAGES <package> MIDDLEWARES ros2)' "
            << "in your build system!" << std::endl;

    _logger << utils::Logger::Level::DEBUG
            << " -- Checked locations (these files did not exist or could not be accessed):\n";

    for (const std::string& checked_path : checked_paths)
    {
        _logger << "\t- " << checked_path << "\n";
    }
    _logger << std::endl;
}

//==============================================================================
SystemHandle::SystemHandle()
    : _logger("is::sh::ROS2")
{
}

//==============================================================================
bool SystemHandle::configure(
        const core::RequiredTypes& types,
        const YAML::Node& configuration,
        TypeRegistry& type_registry)
{
    const int argc = 1;
    const char* argv[argc];
    bool success = true;
    argv[0] = "is_ros2";

    if (!rclcpp::ok())
    {
        rclcpp::init(argc, argv);
    }

    std::string ns = "";
    if (const YAML::Node namespace_node = configuration["namespace"])
    {
        ns = namespace_node.as<std::string>("");
    }

    std::stringstream node_name_ss;
    node_name_ss << "is_ros2_node_" << rand();
    std::string name = node_name_ss.str();

    if (const YAML::Node name_node = configuration["node_name"])
    {
        name = name_node.as<std::string>("");
    }

    if (const YAML::Node domain_node = configuration["domain"])
    {
        if (!configuration["node_name"])
        {
            _logger << utils::Logger::Level::WARN
                    << "It is recommended to set the 'node_name' attribute "
                    << "in the YAML file when using the 'domain' option. "
                    << "The default node name will be '"
                    << name << "'." << std::endl;
        }
        // TODO(@jamoralp) Warn if not set a custom node name unique for each node.
        std::string previous_domain;
        if (getenv("ROS_DOMAIN_ID") != nullptr)
        {
            previous_domain = getenv("ROS_DOMAIN_ID");
        }

        std::string domain = domain_node.as<std::string>();
        success &= set_platform_env("ROS_DOMAIN_ID", domain.c_str(), true);

        if (!success)
        {
            _logger << utils::Logger::Level::ERROR
                    << "Failed to set ROS_DOMAIN_ID to " << domain << std::endl;

            return false;
        }
        /**
         * Since ROS2 Foxy, there is one participant per context.
         * Thus, to ensure isolation between nodes,
         * each node must be created in a different context.
         */

        rclcpp::InitOptions init_options;
        if (rcl_logging_rosout_enabled())
        {
            init_options.auto_initialize_logging(false);
        }

        const char* context_argv[1];
        const std::string context_name("is_ros2_context_" + name);
        context_argv[0] = context_name.c_str();

        _context = std::make_shared<rclcpp::Context>();
        _context->init(1, context_argv, init_options);

        _executor_options.context = _context;

        _node_options = std::make_shared<rclcpp::NodeOptions>();
        _node_options->context(_context);

        _node = std::make_shared<rclcpp::Node>(name, ns, *_node_options);

        _logger << utils::Logger::Level::INFO
                << "Created node '" << ns << "/" << name << "' with Domain ID: "
                << _node_options->get_rcl_node_options()->domain_id << std::endl;

        if (previous_domain.empty())
        {
            success &= unset_platform_env("ROS_DOMAIN_ID");
        }
        else
        {
            success &= set_platform_env("ROS_DOMAIN_ID", previous_domain.c_str(), true);
        }
    }
    else
    {
        // Using default DOMAIN_ID, the node can be added to the default context.
        _logger << utils::Logger::Level::INFO
                << "Created node '" << ns << "/" << name << "'" << std::endl;

        _node = std::make_shared<rclcpp::Node>(name, ns);
    }

    // TODO(MXG): Allow the type of executor to be specified by the configuration
    _executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(_executor_options);

    auto register_type = [&](const std::string& type_name) -> bool
            {
                xtypes::DynamicType::Ptr type = Factory::instance().create_type(type_name);

                if (type.get() == nullptr)
                {
                    _logger << utils::Logger::Level::ERROR
                            << "Failed to register the required DynamicType '"
                            << type_name << "'" << std::endl;

                    return false;
                }
                else
                {
                    _logger << utils::Logger::Level::DEBUG
                            << "Registered the required DynamicType '"
                            << type_name << "'" << std::endl;

                    type_registry.emplace(type_name, std::move(type));
                    return true;
                }
            };

    core::Search search("ros2");

    /**
     * Register ROS 2 messages types within the TypeRegistry.
     */
    for (const std::string& type_name : types.messages)
    {
        std::vector<std::string> checked_paths;
        const std::string msg_mix_path =
                search.find_message_mix(type_name, &checked_paths);

        if (msg_mix_path.empty())
        {
            print_missing_mix_file("message", type_name, checked_paths);
            success = false;
            continue;
        }

        if (!core::Mix::from_file(msg_mix_path).load())
        {
            _logger << utils::Logger::Level::ERROR
                    << "Failed to load extension for message type '"
                    << type_name << "' using mix file: " << msg_mix_path << std::endl;

            success = false;
            continue;
        }
        else
        {
            _logger << utils::Logger::Level::DEBUG
                    << "Loaded middleware interface extension for message type '"
                    << type_name << "' using mix file: " << msg_mix_path << std::endl;

            success = register_type(type_name);
        }
    }

    /**
     * Register ROS 2 services types within the TypeRegistry.
     */
    for (const std::string& type_name : types.services)
    {
        const std::string library_name = type_name.substr(0, type_name.find(":"));
        std::vector<std::string> checked_paths;

        const std::string srv_mix_path =
                search.find_service_mix(library_name, &checked_paths);

        if (srv_mix_path.empty())
        {
            print_missing_mix_file("service", library_name, checked_paths);
            success = false;
            continue;
        }

        if (!core::Mix::from_file(srv_mix_path).load())
        {
            _logger << utils::Logger::Level::ERROR
                    << "Failed to load extension for service type '"
                    << type_name << "' using mix file: " << srv_mix_path << std::endl;

            success = false;
            continue;
        }
        else
        {
            _logger << utils::Logger::Level::DEBUG
                    << "Loaded middleware interface extension for service type '"
                    << type_name << "' using mix file: " << srv_mix_path << std::endl;

            success = register_type(type_name);
        }
    }

    return success;
}

//==============================================================================
bool SystemHandle::okay() const
{
    if (_node)
    {
        return rclcpp::ok(_context);
    }

    return false;
}

//==============================================================================
bool SystemHandle::spin_once()
{
    _executor->spin_node_once(_node, std::chrono::milliseconds(100));
    return rclcpp::ok(_context);
}

//==============================================================================
SystemHandle::~SystemHandle()
{
    _subscriptions.clear();
    _client_proxies.clear();

    rclcpp::shutdown(_context);
}

//==============================================================================
bool SystemHandle::subscribe(
        const std::string& topic_name,
        const xtypes::DynamicType& message_type,
        SubscriptionCallback callback,
        const YAML::Node& configuration)
{
    auto subscription = Factory::instance().create_subscription(
        message_type, *_node, topic_name, std::move(callback),
        parse_rmw_qos_configuration(configuration));

    if (!subscription)
    {
        _logger << utils::Logger::Level::ERROR
                << "Failed to create subscription for topic '" << topic_name
                << "' with type '" << message_type.name() << "' on node '"
                << _node->get_namespace() << _node->get_name() << "'"
                << ". The requested subscription has not been registered within the "
                << "subscription factory!" << std::endl;

        return false;
    }
    else
    {
        _subscriptions.emplace_back(std::move(subscription));

        _logger << utils::Logger::Level::INFO
                << "Created subscription for topic '" << topic_name << "' with type '"
                << message_type.name() << "' on node '" << _node->get_namespace()
                << _node->get_name() << "'" << std::endl;

        return true;
    }
}

//==============================================================================
std::shared_ptr<TopicPublisher> SystemHandle::advertise(
        const std::string& topic_name,
        const xtypes::DynamicType& message_type,
        const YAML::Node& configuration)
{
    std::shared_ptr<TopicPublisher> publisher;

    if (topic_name.find('{') != std::string::npos)
    {
        // If the topic name contains a curly brace, we must assume that it needs
        // runtime substitutions.
        publisher = make_meta_publisher(
            message_type, *_node, topic_name,
            parse_rmw_qos_configuration(configuration),
            configuration);
    }
    else
    {
        publisher = Factory::instance().create_publisher(
            message_type, *_node, topic_name,
            parse_rmw_qos_configuration(configuration));
    }

    if (nullptr != publisher)
    {
        _logger << utils::Logger::Level::INFO
                << "Created publisher for topic '" << topic_name << "' with type '"
                << message_type.name() << "' on node '" << _node->get_namespace()
                << _node->get_name() << "'" << std::endl;
    }
    else
    {
        _logger << utils::Logger::Level::ERROR
                << "Failed to create publisher for topic '" << topic_name
                << "' with type '" << message_type.name() << "' on node '"
                << _node->get_namespace() << _node->get_name()
                << "'. The requested publisher has not been registered "
                << "within the publisher factory!" << std::endl;
    }

    return publisher;
}

//==============================================================================
bool SystemHandle::create_client_proxy(
        const std::string& service_name,
        const xtypes::DynamicType& service_type,
        RequestCallback callback,
        const YAML::Node& configuration)
{
    auto client_proxy = Factory::instance().create_client_proxy(
        service_type.name(), *_node, service_name, callback,
        parse_rmw_qos_configuration(configuration));

    if (!client_proxy)
    {
        _logger << utils::Logger::Level::ERROR
                << "Failed to create service client for service '" << service_name
                << "' with type '" << service_type.name() << "' on node '"
                << _node->get_namespace() << _node->get_name()
                << "'. The requested service client has not been registered "
                << "within the service client factory!" << std::endl;

        return false;
    }
    else
    {
        _logger << utils::Logger::Level::INFO
                << "Created service client for service '" << service_name
                << "' with type '" << service_type.name() << "' on node '"
                << _node->get_namespace() << _node->get_name() << "'" << std::endl;

        _client_proxies.emplace_back(std::move(client_proxy));
        return true;
    }
}

//==============================================================================
std::shared_ptr<ServiceProvider> SystemHandle::create_service_proxy(
        const std::string& service_name,
        const xtypes::DynamicType& service_type,
        const YAML::Node& configuration)
{
    auto server_proxy = Factory::instance().create_server_proxy(
        service_type.name(), *_node, service_name,
        parse_rmw_qos_configuration(configuration));

    if (!server_proxy)
    {
        _logger << utils::Logger::Level::ERROR
                << "Failed to create service server for service '" << service_name
                << "' with type '" << service_type.name() << "' on node '"
                << _node->get_namespace() << _node->get_name()
                << "'. The requested service server has not been registered "
                << "within the service server factory!" << std::endl;
    }
    else
    {
        _logger << utils::Logger::Level::INFO
                << "Created service server for service '" << service_name
                << "' with type '" << service_type.name() << "' on node '"
                << _node->get_namespace() << _node->get_name() << "'" << std::endl;
    }

    return server_proxy;
}

} //  namespace ros2
} //  namespace sh
} //  namespace is
} //  namespace eprosima

//==============================================================================
IS_REGISTER_SYSTEM("ros2", eprosima::is::sh::ros2::SystemHandle)
