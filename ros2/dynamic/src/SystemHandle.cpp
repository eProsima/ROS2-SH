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

namespace xtypes = eprosima::xtypes;

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
        , allow_internal(false)
        , logger_("is::sh::ROS2_Dynamic")
    {
    }

    ~SystemHandle()
    {
        // Iterate over all the packages created during the execution and remove all the associated data from the
        // temporal folder
        std::filesystem::path tmp = std::filesystem::temp_directory_path();
        for (auto pkg_name: package_names)
        {
            std::filesystem::remove_all(tmp / pkg_name);
        }
    }

    bool configure(
            const core::RequiredTypes& /*types*/,
            const YAML::Node& configuration,
            TypeRegistry& type_registry) override
    {
        /*
         * If any type different from the ROS 2 builtin ones want to be used, it need to
         * be defined in the 'types' section of the YAML file.
         */

        if (configuration["namespace"])
        {
            namespace_ = configuration["namespace"].as<std::string>();
        }

        if (configuration["using"])
        {
            if (!add_types_to_registry(configuration["using"], type_registry))
            {
                logger_ << utils::Logger::Level::ERROR
                        << "Failed to register the types." << std::endl;

                return false;
            }
        }

        // If true allows the communication between internal publishers and subscribers in the same topic
        if (configuration["allow_internal"])
        {
            allow_internal = configuration["allow_internal"].as<bool>();
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
            const YAML::Node& configuration) override
    {
        try
        {
            // Apply the name mangling to the topic name before creating the subscriber
            std::string topic_name_mangling = get_ros2_topic_name(topic_name);
            auto subscriber = std::make_shared<Subscriber>(
                participant_.get(), topic_name_mangling, message_type, callback, configuration);

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

    bool is_internal_message(
            void* filter_handle)
    {
        ::fastdds::dds::SampleInfo* sample_info = static_cast<::fastdds::dds::SampleInfo*>(filter_handle);

        auto sample_writer_guid = fastrtps::rtps::iHandle2GUID(sample_info->publication_handle);

        if (sample_writer_guid.guidPrefix == participant_->get_dds_participant()->guid().guidPrefix && !allow_internal)
        {
            if (utils::Logger::Level::DEBUG == logger_.get_level())
            {
                for (const auto& publisher : publishers_)
                {
                    if (sample_writer_guid == fastrtps::rtps::iHandle2GUID(publisher->get_dds_instance_handle()))
                    {
                        logger_ << utils::Logger::Level::DEBUG
                                << "Received internal message from publisher '"
                                << publisher->topic_name() << "', ignoring it..." << std::endl;

                        break;
                    }
                }
            }
            // This is a message published FROM Integration Service. Discard it.
            return true;
        }

        return false;
    }

    std::shared_ptr<TopicPublisher> advertise(
            const std::string& topic_name,
            const xtypes::DynamicType& message_type,
            const YAML::Node& configuration) override
    {
        try
        {
            // Apply the name mangling to the topic name before creating the publisher
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

    bool add_types_to_registry(
        const YAML::Node& configuration,
        TypeRegistry& type_registry)
    {
        std::map<std::pair<std::string, std::string>, std::string> type_paths; //<Package, Type>, Path

        logger_ << utils::Logger::Level::INFO
                << "Adding types to Type Registry for ROS 2 Dynamic SystemHandle" << std::endl;

        for (auto& conf : configuration)
        {
            std::string type = conf.as<std::string>();
            std::size_t found = type.find("/");
            // The type introduced is a package. Register all the types inside that package
            if (found == std::string::npos)
            {
                std::string path = "/opt/ros/" + ROS2_DISTRO + "/share/" + type + "/msg";

                for (const auto & entry : std::filesystem::directory_iterator(path))
                {
                    if (std::filesystem::is_regular_file(entry) && entry.path().extension() == ".idl")
                    {
                        logger_ << utils::Logger::Level::DEBUG << "The ROS 2 package '"
                                << type << "' was added to the type registry." << std::endl;

                        // Add the path to the map along with the package_name and type_name
                        type_paths.emplace(std::make_pair(type, entry.path().stem().string()), entry.path().string());
                    }
                }
            }
            // If it is a message instead of a package, just insert it
            else
            {
                std::string pkg = type.substr(0, found);
                std::string type_name = type.substr(found + 1);
                std::string path = "/opt/ros/" + ROS2_DISTRO + "/share/" + pkg + "/msg/" + type_name + ".idl";

                if (std::filesystem::exists(path))
                {
                    logger_ << utils::Logger::Level::DEBUG << "The ROS 2 message '"
                            << type << "' was added to the type registry." << std::endl;

                    // Add the path to the map along with the package_name and type_name
                    type_paths.emplace(std::make_pair(pkg, type_name), path);
                }
                else
                {
                    logger_ << utils::Logger::Level::WARN << "The type '" << type_name
                            << "' doesn't exists within the package '" << pkg
                            << "'. It will be ignored." << std::endl;
                }
            }
        }

        // Iterate over all the entries of the map, parse the file associated using the xtypes parser and add a new
        // entry in the type_registry
        for (auto const& [type, path] : type_paths)
        {
            xtypes::idl::Context context;
            context.allow_keyword_identifiers = true;
            context.include_paths.push_back("/opt/ros/" + ROS2_DISTRO + "/share");
            xtypes::idl::parse_file(path, context);

            if (context.success)
            {
                xtypes::DynamicType::Ptr dtype = context.module().submodule(type.first)->type(type.second, true);
                if (dtype.get())
                {
                    type_registry.emplace(type.first + "::msg::" + type.second, std::move(dtype));
                }
            }
        }

        return true;
    }

    void replace_all_string(
            std::string& str,
            const std::string& from,
            const std::string& to) const
    {
        // Replace all the occurrences of a substring with the second one
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
        // Check that the types YAML node is not empty
        if (!types_node)
        {
            return true;
        }

        // Storage for the IDL paths that are needed to construct the xtype creation
        std::vector<std::string> include_paths;
        if (types_node["paths"])
        {
            for (const auto& path : types_node["paths"])
            {
                // Insert each of the paths in the vector
                include_paths.push_back(path.as<std::string>());

                if (logger_.get_level() == is::utils::Logger::Level::DEBUG)
                {
                    // Check if there are paths to custom IDLs
                    std::regex reg("/opt/ros/([a-z])+/share/*");
                    if (!std::regex_match (path.as<std::string>(), reg))
                    {
                        logger_ << utils::Logger::Level::DEBUG
                                << "Added path for custom IDL." << std::endl;
                    }
                }
            }
        }

        if(types_node["idls"])
        {
            // Iterate over each of the IDLs set on the YAML configuration file
            for (const auto& entry : types_node["idls"])
            {
                 logger_ << utils::Logger::Level::DEBUG
                                << "IDL: " << Dump(entry) << std::endl;

                // Set to store the modules that corresponds to ROS 2 messages
                std::set<std::string> ros2_modules;
                // Vector to include the paths to custom IDL files
                std::vector<std::string> custom_include_paths;
                bool custom_includes = false;

                // Retrieve the include clauses from the IDL
                std::regex incl_reg("#\\s*include\\s*[<\"][^>\"]*[>\"]");
                std::string idl = entry.as<std::string>();
                // This is due to the way libyaml-cpp treats the string lists
                replace_all_string(idl, "#include", "\n#include");

                logger_ << utils::Logger::Level::DEBUG
                                << "IDL string" << idl << std::endl;

                // Apply the include regex
                std::sregex_iterator iter(idl.begin(), idl.end(), incl_reg);
                std::sregex_iterator end;

                logger_ << utils::Logger::Level::DEBUG
                        << "Searching include clauses..." << std::endl;

                // Iterate over all the include regex matches
                while (iter != end)
                {
                    for(unsigned i = 0; i < iter->size(); ++i)
                    {
                        // On each include extracts the package_name
                        std::string incl = (*iter)[i];
                        logger_ << utils::Logger::Level::DEBUG
                                << "Include Clause: " << incl << std::endl;

                        // Regex to extract the first part of the include
                        std::regex pkg_reg("[<\"][^>\"/]*[/]");
                        std::cmatch cm;
                        std::regex_search(incl.c_str(), cm, pkg_reg);
                        std::string pkg = cm[0];

                        // Check that the regex search success
                        if (!pkg.empty())
                        {
                            // Remove from the package name the first character (<) and the last one (/)
                            pkg = pkg.substr(1, pkg.length() - 2);
                            logger_ << utils::Logger::Level::DEBUG
                                    << "Include Package: " << pkg << std::endl;

                            // Check if the package has a corresponding folder within ROS 2 installation
                            struct stat sb;
                            std::string dir = "/opt/ros/" + ROS2_DISTRO + "/include/" + pkg;
                            if (stat(dir.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))
                            {
                                // If the folder is found, it is inserted in the ros2_modules set
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

                        // If there are custom includes, obtain the complete path to copy it later to the new ROS 2
                        // package so that it can be found during the execution
                        if (custom_includes)
                        {
                            // Extract the path from the include clause
                            std::regex path_reg("[<\"][^>\"]*[\">]");
                            std::cmatch cm1;
                            std::regex_search(incl.c_str(), cm1, path_reg);
                            std::string in_path = cm1[0];
                            in_path = in_path.substr(1, in_path.length() - 2);

                            // Check if the file can be found within any of the paths set in the YAML config file
                            for (const auto& path : include_paths)
                            {
                                logger_ << utils::Logger::Level::DEBUG
                                        << "Looking for file " << in_path << " within path " << path << std::endl;

                                // If the file exists in the path, it is stored in the custom_include_paths vector
                                // which will be used later to copy the file
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

                logger_ << utils::Logger::Level::INFO
                        << "Parsing IDL" << std::endl;

                // Configures the xtypes parser options
                xtypes::idl::Context context;
                // Allow setting keywords as identifier within the IDL representation
                context.allow_keyword_identifiers = true;
                if (!include_paths.empty())
                {
                    // Pass all the YAML paths, so that the parser can find the IDLs
                    context.include_paths = include_paths;
                }

                // Obtain the xtypes representation of the IDL
                xtypes::idl::parse(entry.as<std::string>(), context);

                // If the IDL parse ends successfully continue with the type preprocess
                if (context.success)
                {
                    // Due to the ROS 2 convention any IDL type must have two modules (package_name and msg/srv)
                    size_t modules = context.module().submodule_size() - ros2_modules.size();
                    logger_ << utils::Logger::Level::DEBUG
                            << "Number of modules: " << modules << std::endl;

                    // If there isn't any module, it throws an error
                    if (modules < 1)
                    {
                        logger_ << utils::Logger::Level::ERROR
                                << "The type is not declared within an IDL module."
                                << " Please follow the ROS 2 naming convention."
                                << std::endl;

                        return false;
                    }
                    else
                    {
                        bool is_success = true;
                        // Iterates over all the modules found by the xtypes parser
                        context.module().for_each_submodule([&] (const xtypes::idl::Module& mod)
                            {
                                // If the module is inside the ROS 2 modules' set it is discarded
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

                                // Check whether it has a submodule that corresponds to msg or srv
                                if (!mod.has_submodule("msg") && !mod.has_submodule("srv"))
                                {
                                    logger_ << utils::Logger::Level::ERROR
                                            << "The module '" << mod.name()
                                            << "' must be followed by a 'msg' or 'srv' submodule prior to declaring the type."
                                            << std::endl;
                                    is_success = false;
                                    return;
                                }

                                // Create a new folder with the name of the ROS 2 package within the temporal location
                                // and inside it another called msg/srv to store the .idl files that will be created
                                std::filesystem::path tmp = std::filesystem::temp_directory_path();
                                //TODO: Change when the services are implemented
                                std::filesystem::create_directories(tmp / mod.name() / "msg");

                                // Save the package_name for cleaning the temporal location when the execution is ended
                                package_names.insert(mod.name());

                                // Gets from the specific submodule a map with the name and the dynamic type associated
                                auto all_types = context.module().submodule(mod.name()).get()->get_all_types();

                                for (auto [name, dtype] : all_types)
                                {
                                    // Check if that the type is an Structure (ROS 2 restriction)
                                    if (dtype.get()->kind() != xtypes::TypeKind::STRUCTURE_TYPE)
                                    {
                                        logger_ << utils::Logger::Level::ERROR
                                            << "[" << name
                                            << "] is not a structure."
                                            << std::endl;
                                        is_success = false;
                                        return;
                                    }

                                    // Check that the message name follows the ROS2 naming convention
                                    std::regex type_reg("[A-Z]([a-zA-Z0-9])*");
                                    if (!std::regex_match(name, type_reg))
                                    {
                                        logger_ << utils::Logger::Level::ERROR
                                            << "The message name [" << name
                                            << "] doesn't follow the ROS2 naming convention."
                                            << std::endl;
                                        is_success = false;
                                        return;
                                    }
                                };

                                // Copy the custom IDLs to the created folder using the previously stored paths
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

                            }, false);

                            // Due to a rosidl restriction each IDL file can only contain one structure definition.
                            // The generate function take the module xtypes objects and generate the associated IDL
                            // strings. Each map entry represents the name of the type and the generated IDL.
                            std::map<std::string, std::string> resulting_idl;
                            auto m_idl = eprosima::xtypes::idl::generate(
                                static_cast<const xtypes::idl::Module&>
                                (context.module()), &resulting_idl);


                            std::string depends = "";

                            // The Xtypes generator generates code also for the ROS2 types, which is not neccessary.
                            // Each of them are marked as removed
                            std::set<std::string> remove;
                            if (!ros2_modules.empty())
                            {
                                for (auto& pair : resulting_idl)
                                {
                                    if (ros2_modules.find(pair.first.substr(0, pair.first.find("::"))) != ros2_modules.end())
                                    {
                                        remove.insert(pair.second);
                                    }
                                }
                            }

                            // Transform the dependencies into include clauses
                            for (auto& pair : resulting_idl)
                            {
                                // The generate function returns to types of entries, the ones having the type name and
                                // its IDL and the one with the dependencies associated to a specific type

                                // If the pair corresponds with a type_name-IDL entry and it is not a ROS 2 message
                                if (pair.first.find(":dependencies") == std::string::npos
                                    && remove.find(pair.second) == remove.end())
                                {
                                    std::set<std::string> dependencies;
                                    logger_ << utils::Logger::Level::DEBUG
                                            << "[" << pair.first << "]";

                                    // If the IDL associated to a ROS 2 message is within the current type IDL, it is
                                    // removed before continuing
                                    for (auto& r_str : remove)
                                    {
                                        auto pos = pair.second.find(r_str);
                                        if (pos != std::string::npos)
                                        {
                                            pair.second.erase(pos, r_str.length());
                                        }

                                    }

                                    // Find within the map the dependencies entry associated with the current type
                                    auto it = resulting_idl.find(pair.first + ":dependencies");
                                    if (it != resulting_idl.end())
                                    {
                                        // Generate the include clauses based on the dependencies and add them to the
                                        // type associated IDL
                                        std::stringstream ss(resulting_idl[pair.first + ":dependencies"]);
                                        std::string str;
                                        std::string includes;
                                        while (getline(ss, str, ',')) {
                                            std::string package = str.substr(0, str.find("::"));
                                            // If the dependency corresponds with the same package as the current IDL
                                            // don't insert it. It will generate a circular dependency.
                                            if (pair.first.find(package) == std::string::npos)
                                            {
                                                dependencies.insert(package);
                                            }
                                            includes += "#include <" + str + ".idl>\n";
                                        }
                                        replace_all_string(includes, "::", "/");
                                        resulting_idl.erase(it);
                                        resulting_idl[pair.first] = includes + "\n" + pair.second;
                                    }

                                    logger_ << pair.second << std::endl;

                                    // Generate a new file within the previously created folder with the type name and
                                    // the IDL as content
                                    std::string struct_path = pair.first;
                                    replace_all_string(struct_path, "::", "/");
                                    std::ofstream idlfile ("/tmp/" + struct_path + ".idl");
                                    idlfile << pair.second << std::endl;
                                    idlfile.close();

                                    // Fill the command to call the generator.bash file
                                    // This parameter corresponds to the dependencies associated to a specific package
                                    if (!dependencies.empty())
                                    {
                                        std::ostringstream dep_stream;
                                        std::copy(dependencies.begin(), dependencies.end(),
                                            std::ostream_iterator<std::string>(dep_stream, ";"));
                                        depends += "--" + pair.first.substr(0, pair.first.find("::"))
                                            + " \"" + dep_stream.str() + "\" ";
                                    }
                                }
                            }

                            // The rest of parameters correspond to the name of the package and the installation path
                            std::ostringstream pkg_stream;
                            std::copy(package_names.begin(), package_names.end(), std::ostream_iterator<std::string>(pkg_stream, " "));
                            const std::string package_name = "--package_name \"" + pkg_stream.str() + "\" ";
                            const std::string path = "--install_path /opt/ros/" + ROS2_DISTRO;

                            logger_ << is::utils::Logger::Level::INFO
                                    << "Generating ROS2 Type Support for package: " << package_name
                                    << std::endl;

                            // Then compound the whose command to call the generator.bash
                            std::string command = "exec bash /tmp/is_ros2_sh/generator.bash " + package_name + " " + path + " " + depends;

                            logger_ << utils::Logger::Level::DEBUG
                                    << "Command: " << command << std::endl;

                            // Call the bash that generates the ROS 2 type support for the IDL types
                            FILE* pipe = popen(command.c_str(), "r");
                            if (!pipe)
                            {
                                logger_ << utils::Logger::Level::ERROR
                                        << " Failed to execute command: " << command
                                        << std::endl;
                                return false;
                            }

                            char buffer[128];
                            std::string output = "";
                            while(!feof(pipe)) {
                                if(fgets(buffer, 128, pipe) != NULL)
                                    output += buffer;
                            }

                            // Redirect the bash output to the console
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

                                    return false;
                                }
                                else if (0 == WEXITSTATUS(st))
                                {
                                    logger_ << is::utils::Logger::Level::INFO
                                            << "ROS2 Type Supports generation finished for package '"
                                            << package_name << "', installedin path "
                                            << path << std::endl;
                                }

                            }

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

    bool allow_internal;

    std::set<std::string> package_names;

    utils::Logger logger_;
};



} //  namespace ros2
} //  namespace sh
} //  namespace is
} //  namespace eprosima

IS_REGISTER_SYSTEM("ros2_dynamic", eprosima::is::sh::ros2::SystemHandle)
