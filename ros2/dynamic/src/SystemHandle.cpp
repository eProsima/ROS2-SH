/*
 * Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <is/systemhandle/SystemHandle.hpp>
#include <is/utils/Log.hpp>

#include <is/sh/ros2/Participant.hpp>
#include <is/sh/ros2/Publisher.hpp>
#include <is/sh/ros2/Subscriber.hpp>
#include <is/sh/ros2/Conversion.hpp>

#include <is/sh/ros2/config.hpp>

#include <iostream>
#include <thread>
#include <filesystem>
#include <fstream>
#include <sys/stat.h>

#include <semaphore.h>
#include <fcntl.h>
#include <unistd.h>

namespace eprosima {
namespace is {
namespace sh {
namespace ros2 {

/**
 * @class SystemHandle
 *        This class represents a full *Integration Service* system handle or plugin for the *ROS2*
 *        middleware, using eProsima's implementation, <a href="https://fast-dds.docs.eprosima.com/en/latest/">
 *        Fast DDS</a>.
 *
 *        This class inherits from is::FullSystem, so to implement publisher, subscriber
 *        operations for the *Integration Service*.
 *
 */
class SystemHandle : public virtual FullSystem
{
public:

    SystemHandle()
        : FullSystem()
        , logger_("is::sh::ROS2_Dynamic")
    {
    }

    ~SystemHandle()
    {
        std::filesystem::path tmp = std::filesystem::temp_directory_path();
        for (auto pkg_name: package_names)
        {
            std::filesystem::remove_all(tmp / pkg_name);
        }
    }

    bool configure(
            const core::RequiredTypes& /*types*/,
            const YAML::Node& configuration,
            TypeRegistry& /*type_registry*/) override
    {
        /*
         * The ROS 2 Dynamic SH doesn't define new types.
         * Needed types will be defined in the 'types' section of the YAML file, and hence,
         * already registered in the 'TypeRegistry' by the *Integration Service core*.
         */

        if(configuration["namespace"])
        {
            namespace_ = configuration["namespace"].as<std::string>();
        }

        try
        {
            participant_ = std::make_unique<Participant>(configuration);
        }
        catch (ROS2MiddlewareException& e)
        {
            e.from_logger << utils::Logger::Level::ERROR << e.what() << std::endl;
            return false;
        }

        logger_ << utils::Logger::Level::INFO << "Configured!" << std::endl;

        return true;
    }

    bool okay() const override
    {
        return (nullptr != participant_->get_dds_participant());
    }

    bool spin_once() override
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);
        return okay();
    }

    bool subscribe(
            const std::string& topic_name,
            const xtypes::DynamicType& message_type,
            SubscriptionCallback* callback,
            const YAML::Node& /* configuration */) override
    {
        try
        {
            std::string topic_name_mangling = get_ros2_topic_name(topic_name);
            auto subscriber = std::make_shared<Subscriber>(
                participant_.get(), topic_name_mangling, message_type, callback);

            subscribers_.emplace_back(std::move(subscriber));

            logger_ << utils::Logger::Level::INFO
                    << "Subscriber created for topic '" << topic_name_mangling << "', with type '"
                    << message_type.name() << "'" << std::endl;

            return true;
        }
        catch (ROS2MiddlewareException& e)
        {
            e.from_logger << utils::Logger::Level::ERROR << e.what() << std::endl;
            return false;
        }
    }

    std::shared_ptr<TopicPublisher> advertise(
            const std::string& topic_name,
            const xtypes::DynamicType& message_type,
            const YAML::Node& configuration) override
    {
        try
        {
            std::string topic_name_mangling = get_ros2_topic_name(topic_name);
            auto publisher = std::make_shared<Publisher>(
                participant_.get(), topic_name_mangling, message_type, configuration);
            publishers_.emplace_back(std::move(publisher));

            logger_ << utils::Logger::Level::INFO
                    << "Publisher created for topic '" << topic_name_mangling << "', with type '"
                    << message_type.name() << "'" << std::endl;

            return publishers_.back();
        }
        catch (ROS2MiddlewareException& e)
        {
            e.from_logger << utils::Logger::Level::ERROR << e.what() << std::endl;
            return std::shared_ptr<TopicPublisher>();
        }
    }

    void replace_all_string(
            std::string& str,
            const std::string& from,
            const std::string& to) const
    {
        size_t froms = from.size();
        size_t tos = to.size();
        size_t pos = str.find(from);
        while (pos != std::string::npos)
        {
            str.replace(pos, froms, to);
            pos = str.find(from, pos + tos);
        }
    }

    bool preprocess_types(
        const YAML::Node& types_node) override
    {
        std::vector<std::string> include_paths;
        if (types_node["paths"])
        {
            // Check if there are paths to custom idls
            std::regex reg("/opt/ros/([a-z])+/share/*");
            for (const auto& path : types_node["paths"])
            {
                if (!std::regex_match (path.as<std::string>(), reg))
                {
                    logger_ << utils::Logger::Level::DEBUG
                            << "Added path for custom IDL." << std::endl;
                }
                include_paths.push_back(path.as<std::string>());
            }
        }

        if(types_node["idls"])
        {
            for (const auto& entry : types_node["idls"])
            {
                 logger_ << utils::Logger::Level::DEBUG
                                << "IDL: " << Dump(entry) << std::endl;

                std::set<std::string> ros2_modules;
                std::vector<std::string> custom_include_paths;
                bool custom_includes = false;
                // Retrieve the include clauses from the IDL
                std::regex incl_reg("#\\s*include\\s+[<\"][^>\"]*[>\"]");
                std::string idl = entry.as<std::string>();
                replace_all_string(idl, "#include", "\n#include");
                std::sregex_iterator iter(idl.begin(), idl.end(), incl_reg);
                std::sregex_iterator end;

                logger_ << utils::Logger::Level::DEBUG
                        << "Searching include clauses..." << std::endl;

                while (iter != end)
                {
                    for(unsigned i = 0; i < iter->size(); ++i)
                    {
                        // On each include extracts the package_name
                        std::string incl = (*iter)[i];
                        logger_ << utils::Logger::Level::DEBUG
                                << "Include Clause: " << incl << std::endl;

                        std::regex pkg_reg("[<\"][^>\"/]*[/]");
                        std::cmatch cm;
                        std::regex_search(incl.c_str(), cm, pkg_reg);
                        std::string pkg = cm[0];

                        if (!pkg.empty())
                        {
                            pkg = pkg.substr(1, pkg.length() - 2);
                            logger_ << utils::Logger::Level::DEBUG
                                    << "Include Package: " << pkg << std::endl;

                            // Check if the package has a corresponding folder within ros installation
                            struct stat sb;
                            std::string dir = "/opt/ros/" + ROS2_DISTRO + "/include/" + pkg;
                            if (stat(dir.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))
                            {
                                ros2_modules.insert(pkg);
                            }
                            else
                            {
                                custom_includes = true;
                            }
                        }
                        else
                        {
                            custom_includes = true;
                        }

                        if (custom_includes)
                        {
                            std::regex path_reg("[<\"][^>\"]*[\">]");
                            std::cmatch cm1;
                            std::regex_search(incl.c_str(), cm1, path_reg);
                            std::string in_path = cm1[0];
                            in_path = in_path.substr(1, in_path.length() - 2);
                            for (const auto& path : include_paths)
                            {
                                logger_ << utils::Logger::Level::DEBUG
                                        << "Looking for file " << in_path << " within path " << path << std::endl;
                                if (std::filesystem::exists(path + in_path))
                                {
                                    logger_ << utils::Logger::Level::DEBUG
                                            << "IDL file found in: " << path + in_path << std::endl;
                                    custom_include_paths.push_back(path + in_path);
                                }
                            }
                        }
                    }
                    ++iter;
                }

                // Obtain the xtypes representation of the IDL
                logger_ << utils::Logger::Level::INFO
                        << "Parsing IDL" << std::endl;

                eprosima::xtypes::idl::Context context;
                context.allow_keyword_identifiers = true;
                if (!include_paths.empty())
                {
                    context.include_paths = include_paths;
                }

                eprosima::xtypes::idl::parse(entry.as<std::string>(), context);

                if (context.success)
                {
                    size_t modules = context.module().submodule_size() - ros2_modules.size();
                    logger_ << utils::Logger::Level::DEBUG
                            << "Number of modules: " << modules << std::endl;

                    if (modules == 0)
                    {
                        logger_ << utils::Logger::Level::ERROR
                                << "The type is not declared within an IDL module. Please follow the ROS 2 naming convention."
                                << std::endl;

                        return false;
                    }
                    else if (modules > 1)
                    {
                        logger_ << utils::Logger::Level::ERROR
                                << "There can only be one module per IDL, add"
                                << " another entry in the YAML `idls` tag." << std::endl;

                        return false;
                    }
                    else
                    {
                        bool is_success = true;
                        context.module().for_each_submodule([&] (const xtypes::idl::Module& mod)
                            {
                                if (ros2_modules.find(mod.name()) != ros2_modules.end())
                                {
                                    return;
                                }

                                // Check that the package_name follows the ROS2 naming convention
                                std::regex reg("[a-z]+(([a-z0-9]*)_?[a-z0-9]+)+");
                                if (!std::regex_match(mod.name(), reg) || mod.name().find("ros") != std::string::npos)
                                {
                                    logger_ << utils::Logger::Level::ERROR
                                            << "The package name [" << mod.name()
                                            << "] doesn't follow the ROS2 naming convention."
                                            << std::endl;
                                    is_success = false;
                                    return;
                                }

                                // Check whether the following submodule is msg or srv
                                if (!mod.has_submodule("msg") && !mod.has_submodule("srv"))
                                {
                                    logger_ << utils::Logger::Level::ERROR
                                            << "The module '" << mod.name()
                                            << "' must be followed by a 'msg' or 'srv' submodule prior to declaring the type."
                                            << std::endl;
                                    is_success = false;
                                    return;
                                }

                                std::filesystem::path tmp = std::filesystem::temp_directory_path();
                                //TODO: Change when the services are implemented
                                std::filesystem::create_directories(tmp / mod.name() / "msg");

                                package_names.insert(mod.name());

                                std::string aux;
                                std::size_t found = idl.rfind("struct");
                                aux = idl.substr(found + 7);
                                found = aux.find("{");
                                aux = aux.substr(0, found);
                                aux.erase(remove_if(aux.begin(), aux.end(), isspace), aux.end());

                                // Get the type corresponding to the last struct inside the current module
                                auto type = context.module().submodule(mod.name()).get()->type(aux, true);

                                // Check if it is a Structure
                                if (type.get()->kind() != xtypes::TypeKind::STRUCTURE_TYPE)
                                {
                                    logger_ << utils::Logger::Level::ERROR
                                        << "[" << aux
                                        << "] is not a structure."
                                        << std::endl;
                                    is_success = false;
                                    return;
                                }

                                // Check that the message name follows the ROS2 naming convention
                                std::regex type_reg("[A-Z]([a-zA-Z0-9])+");
                                if (!std::regex_match(aux, type_reg))
                                {
                                    logger_ << utils::Logger::Level::ERROR
                                        << "The message name [" << aux
                                        << "] doesn't follow the ROS2 naming convention."
                                        << std::endl;
                                    is_success = false;
                                    return;
                                }

                                //TODO: Change when the services are implemented
                                std::ofstream idlfile ("/tmp/" + mod.name() + "/msg/" + aux + ".idl");
                                idlfile << idl << std::endl;
                                idlfile.close();

                                for (const auto& mv_idl : custom_include_paths)
                                {
                                    std::string filename = std::filesystem::path(mv_idl).filename();
                                    std::string path = std::filesystem::path(tmp / mod.name() / "msg" / filename);
                                    logger_ << utils::Logger::Level::DEBUG
                                            << "The file " << filename << " has been copied to " << path
                                            << std::endl;
                                    if (!std::filesystem::exists(path))
                                    {
                                        std::filesystem::copy(mv_idl, path);
                                    }
                                }

                                const std::string package_name = "--package_name " + mod.name();
                                const std::string path = "--install_path /opt/ros/" + ROS2_DISTRO;

                                logger_ << is::utils::Logger::Level::INFO
                                        << "Generating ROS2 Type Support for package: " << package_name
                                        << std::endl;

                                std::string command = "exec bash /tmp/generator.bash " + package_name + " " + path;

                                if (ros2_modules.size() != 0)
                                {
                                    std::ostringstream stream;
                                    std::copy(ros2_modules.begin(), ros2_modules.end(), std::ostream_iterator<std::string>(stream, ";"));
                                    const std::string depends = "--dependencies \"" + stream.str() + "\"";

                                    logger_ << is::utils::Logger::Level::DEBUG
                                        << "ROS2 Type Support Dependencies [" << depends
                                        << "]" << std::endl;

                                    command += " " + depends;
                                }

                                FILE* pipe = popen(command.c_str(), "r");
                                if (!pipe)
                                {
                                    logger_ << utils::Logger::Level::ERROR
                                            << " Failed to execute command: " << command
                                            << std::endl;
                                    is_success = false;
                                    return;
                                }

                                char buffer[128];
                                std::string output = "";
                                while(!feof(pipe)) {
                                    if(fgets(buffer, 128, pipe) != NULL)
                                        output += buffer;
                                }

                                logger_ << is::utils::Logger::Level::DEBUG
                                        << output << std::endl;

                                int st = pclose(pipe);
                                if (WIFEXITED(st))
                                {
                                    if (1 == WEXITSTATUS(st))
                                    {
                                        logger_ << is::utils::Logger::Level::ERROR
                                                << "Failed to generate the Type Support for package '" << package_name
                                                << "'. Make sure you follow all the ROS2 naming and structure convention"
                                                << " rules" << std::endl;

                                        if (logger_.get_level() != is::utils::Logger::Level::DEBUG)
                                        {
                                            logger_ << is::utils::Logger::Level::ERROR
                                                    << output << std::endl;
                                        }

                                        is_success = false;
                                        return;
                                    }
                                    else if (0 == WEXITSTATUS(st))
                                    {
                                        logger_ << is::utils::Logger::Level::INFO
                                                << "ROS2 Type Supports generation finishedfor package '"
                                                << package_name << "', installedin path "
                                                << path << std::endl;
                                    }

                                }

                            }, false);

                        return is_success;
                    }
                }
            }
        }

        return false;
    }

    // bool create_client_proxy(
    //         const std::string& service_name,
    //         const xtypes::DynamicType& type,
    //         RequestCallback* callback,
    //         const YAML::Node& configuration) override
    // {
    //     return create_client_proxy(service_name, type, type, callback, configuration);
    // }

    // bool create_client_proxy(
    //         const std::string& service_name,
    //         const xtypes::DynamicType& request_type,
    //         const xtypes::DynamicType& reply_type,
    //         RequestCallback* callback,
    //         const YAML::Node& configuration) override
    // {
    //     if (clients_.count(service_name) == 0)
    //     {
    //         try
    //         {
    //             auto client = std::make_shared<Client>(
    //                 participant_.get(),
    //                 service_name,
    //                 request_type,
    //                 reply_type,
    //                 callback,
    //                 configuration);

    //             clients_[service_name] = std::move(client);

    //             logger_ << utils::Logger::Level::INFO
    //                     << "Client created for service '" << service_name
    //                     << "', with request_type '" << request_type.name()
    //                     << "' and reply_type '" << reply_type.name() << "'" << std::endl;

    //             return true;

    //         }
    //         catch (ROS2MiddlewareException& e)
    //         {
    //             e.from_logger << utils::Logger::Level::ERROR << e.what() << std::endl;
    //             return false;
    //         }
    //     }

    //     return clients_[service_name]->add_config(configuration, callback);
    // }

    // std::shared_ptr<ServiceProvider> create_service_proxy(
    //         const std::string& service_name,
    //         const xtypes::DynamicType& type,
    //         const YAML::Node& configuration) override
    // {
    //     return create_service_proxy(service_name, type, type, configuration);
    // }

    // std::shared_ptr<ServiceProvider> create_service_proxy(
    //         const std::string& service_name,
    //         const xtypes::DynamicType& request_type,
    //         const xtypes::DynamicType& reply_type,
    //         const YAML::Node& configuration) override
    // {
    //     if (servers_.count(service_name) == 0)
    //     {
    //         try
    //         {
    //             auto server = std::make_shared<Server>(
    //                 participant_.get(),
    //                 service_name,
    //                 request_type,
    //                 reply_type,
    //                 configuration);

    //             servers_[service_name] = std::move(server);

    //             logger_ << utils::Logger::Level::INFO
    //                     << "Server created for service '" << service_name
    //                     << "', with request_type '" << request_type.name()
    //                     << "' and reply_type '" << reply_type.name() << "'" << std::endl;

    //             return servers_[service_name];
    //         }
    //         catch (ROS2MiddlewareException& e)
    //         {
    //             e.from_logger << utils::Logger::Level::ERROR << e.what() << std::endl;
    //             return nullptr;
    //         }
    //     }

    //     if (servers_[service_name]->add_config(configuration))
    //     {
    //         return servers_[service_name];
    //     }
    //     else
    //     {
    //         return nullptr;
    //     }
    // }

private:

    std::string get_ros2_topic_name(
        const std::string& topic_name)
    {
        std::string topic_namespace = "/";
        if (!namespace_.empty())
        {
            if (namespace_.front() == '/')
            {
                topic_namespace = namespace_;
            }
            else
            {
                topic_namespace += namespace_;
            }

            if (namespace_.back() != '/')
            {
                topic_namespace += "/";
            }
        }
        std::string topic_name_mangling = "rt" + topic_namespace + topic_name;

        return topic_name_mangling;
    }

    std::unique_ptr<Participant> participant_;
    std::vector<std::shared_ptr<Publisher> > publishers_;
    std::vector<std::shared_ptr<Subscriber> > subscribers_;
    //std::map<std::string, std::shared_ptr<Client> > clients_;
    //std::map<std::string, std::shared_ptr<Server> > servers_;

    std::string namespace_;

    std::set<std::string> package_names;

    utils::Logger logger_;
};



} //  namespace ros2
} //  namespace sh
} //  namespace is
} //  namespace eprosima

IS_REGISTER_SYSTEM("ros2_dynamic", eprosima::is::sh::ros2::SystemHandle)
