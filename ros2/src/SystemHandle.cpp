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

rmw_qos_profile_t parse_rmw_qos_configuration(
        const YAML::Node& configuration)
{
    rmw_qos_profile_t qos = rmw_qos_profile_default;

    if (configuration)
    {
        logger_ << utils::Logger::Level::DEBUG
                    << "Publisher created with QoS: " << YAML::Dump(config) << std::endl;


        // Configure datawriter QoS according to config node
        if (config["history"])
        {
            if (config["history"]["kind"])
            {
                std::string hist_kind = config["history"]["kind"].as<std::string>();
                if (hist_kind == "KEEP_LAST")
                {
                    qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
                }
                else if (hist_kind == "KEEP_ALL")
                {
                    qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
                }
                else
                {
                    logger_ << utils::Logger::Level::WARN
                            << "History QoS kind is unknown. "
                            << "The valid values are: KEEP_LAST and KEEP_ALL." << std::endl;
                }
            }

            if (config["history"]["depth"])
            {
                qos.depth = config["history"]["depth"].as<int>();
            }
        }

        if (config["reliability"])
        {
            std::string rel_kind = config["reliability"].as<std::string>();
            if (rel_kind == "RELIABLE")
            {
                qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
            }
            else if (rel_kind == "BEST_EFFORT")
            {
                qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
            }
            else
            {
                logger_ << utils::Logger::Level::WARN
                        << "Reliability QoS kind is unknown. "
                        << "The valid values are: RELIABLE and BEST_EFFORT." << std::endl;
            }
        }

        if (config["durability"])
        {
            std::string dur_kind = config["durability"].as<std::string>();
            if (dur_kind == "VOLATILE")
            {
                qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
            }
            else if (dur_kind == "TRANSIENT_LOCAL")
            {
                qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
            }
            else
            {
                logger_ << utils::Logger::Level::WARN
                        << "Durability QoS kind is unknown. "
                        << "The valid values are: VOLATILE and TRANSIENT_LOCAL." << std::endl;
            }
        }

        if (config["deadline"])
        {
            rmw_time_t period;
            if (config["deadline"]["sec"])
            {
                period.sec = config["deadline"]["sec"].as<uint64_t>();
            }

            if (config["deadline"]["nanosec"])
            {
                period.nsec = config["deadline"]["nanosec"].as<uint64_t>();
            }

            qos.deadline = period;
        }

        if (config["lifespan"])
        {
            rmw_time_t duration;
            if (config["lifespan"]["sec"])
            {
                duration.sec = config["lifespan"]["sec"].as<uint64_t>();
            }

            if (config["lifespan"]["nanosec"])
            {
                duration.nsec = config["lifespan"]["nanosec"].as<uint64_t>();
            }

            qos.lifespan = duration;
        }

        if (config["liveliness"])
        {
            if (config["liveliness"]["kind"])
            {
                std::string live_kind = config["liveliness"]["kind"].as<std::string>();
                if (live_kind == "AUTOMATIC")
                {
                    qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
                }
                else if (live_kind == "MANUAL_BY_TOPIC")
                {
                    qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
                }
                else
                {
                    logger_ << utils::Logger::Level::WARN
                            << "Liveliness QoS kind is unknown. "
                            << "The valid values are: AUTOMATIC and MANUAL_BY_TOPIC." << std::endl;
                }
            }

            rmw_time_t lease_duration;
            if (config["liveliness"]["sec"])
            {
                lease_duration.sec = config["liveliness"]["sec"].as<uint64_t>();
            }

            if (config["liveliness"]["nanosec"])
            {
                lease_duration.nsec = config["liveliness"]["sec"].as<uint64_t>();
            }

            qos.liveliness_lease_duration = lease_duration;
        }
    }

    return qos;
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

    auto create_node =
            [&](const size_t domain_id = 0) -> void
            {
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

                init_options.set_domain_id(domain_id);

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
                        << init_options.get_domain_id() << std::endl;
            };

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

        create_node(domain_node.as<size_t>());
    }
    else
    {
        create_node();
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
        SubscriptionCallback* callback,
        const YAML::Node& configuration)
{
    auto subscription = Factory::instance().create_subscription(
        message_type, *_node, topic_name, callback,
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
bool SystemHandle::is_internal_message(
        void* /*filter_handle*/)
{
    // Always return false, since this should be handled by the ROS 2 RMW and the fact that
    // we created the ROS 2 subscriptions with the "ignore_local_publications" flag set to true.
    return false;
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
        RequestCallback* callback,
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
