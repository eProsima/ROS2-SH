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

#include <is/sh/ros2/Participant.hpp>
#include <is/sh/ros2/ROS2MiddlewareException.hpp>
#include <is/sh/ros2/Conversion.hpp>

#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>

#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/xmlparser/XMLProfileManager.h>

#include <sstream>

namespace eprosima {
namespace is {
namespace sh {
namespace ros2 {

Participant::Participant()
    : dds_participant_(nullptr)
    , logger_("is::sh::ROS2_Dynamic::Participant")
{
    build_participant();
}

Participant::Participant(
        const YAML::Node& config)
    : dds_participant_(nullptr)
    , logger_("is::sh::ROS2_Dynamic::Participant")
{
    ::fastdds::dds::DomainId_t domain_id = 0;
    std::string node_name = "default_IS-ROS2-Dynamic-SH_participant";

    if (config["domain"])
    {
        domain_id = config["domain"].as<uint32_t>();
    }
    if (config["node_name"])
    {
        node_name = config["node_name"].as<std::string>();
    }

    build_participant(domain_id, node_name);
}

Participant::~Participant()
{
    if (!dds_participant_->has_active_entities())
    {
        dds_participant_->set_listener(nullptr);

        if (fastrtps::types::ReturnCode_t::RETCODE_OK !=
                ::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(dds_participant_))
        {
            logger_ << utils::Logger::Level::ERROR
                    << "Cannot delete ROS2 Dynamic node yet: it has active entities" << std::endl;
        }
    }
}

void Participant::build_participant(
        const ::fastdds::dds::DomainId_t& domain_id,
        const std::string& participant_name)
{
    ::fastdds::dds::DomainParticipantQos participant_qos = ::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
    participant_qos.name(participant_name);

    // By default use UDPv4 due to communication failures between dockers sharing the network with the host
    // When it is solved in Fast-DDS delete the following lines and use the default builtin transport.
    participant_qos.transport().use_builtin_transports = false;
    auto udp_transport = std::make_shared<::fastdds::rtps::UDPv4TransportDescriptor>();
    participant_qos.transport().user_transports.push_back(udp_transport);

    dds_participant_ = ::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(
        domain_id, participant_qos);

    if (dds_participant_)
    {
        logger_ << utils::Logger::Level::INFO
                << "Created ROS 2 Dynamic node '" << participant_qos.name()
                << "' with default QoS attributes and Domain ID: "
                << domain_id << std::endl;
    }
    else
    {
        std::ostringstream err;
        err << "Error while creating ROS2 Dynamic node '" << participant_qos.name()
            << "' with default QoS attributes and Domain ID: " << domain_id;

        throw ROS2MiddlewareException(logger_, err.str());
    }
}

::fastdds::dds::DomainParticipant* Participant::get_dds_participant() const
{
    return dds_participant_;
}

void Participant::register_dynamic_type(
        const std::string& topic_name,
        const std::string& type_name,
        fastrtps::types::DynamicTypeBuilder* builder)
{
    auto topic_to_type_it = topic_to_type_.find(topic_name);
    if (topic_to_type_it != topic_to_type_.end())
    {
        return; // Already registered.
    }

    auto types_it = types_.find(type_name);
    if (types_.end() != types_it)
    {
        // Type known, add the entry in the map topic->type
        topic_to_type_.emplace(topic_name, type_name);

        logger_ << utils::Logger::Level::DEBUG
                << "Adding type '" << type_name << "' to topic '"
                << topic_name << "'" << std::endl;

        return;
    }

    fastrtps::types::DynamicType_ptr dtptr = builder->build();

    if (dtptr != nullptr)
    {
        auto pair = types_.emplace(type_name, fastrtps::types::DynamicPubSubType(dtptr));
        fastrtps::types::DynamicPubSubType& dynamic_type_support = pair.first->second;

        topic_to_type_.emplace(topic_name, type_name);

        // Check if already registered
        ::fastdds::dds::TypeSupport p_type = dds_participant_->find_type(type_name);

        if (nullptr == p_type)
        {
            dynamic_type_support.setName(type_name.c_str());

            /**
             * The following lines are added here so that a bug with UnionType in
             * Fast DDS Dynamic Types is bypassed. This is a workaround and SHOULD
             * be removed once this bug is solved.
             * Until that moment, the Fast DDS SystemHandle will not be compatible with
             * Fast DDS Dynamic Type Discovery mechanism.
             *
             * More information here: https://eprosima.easyredmine.com/issues/11349
             */
            // WORKAROUND START
            dynamic_type_support.auto_fill_type_information(false);
            dynamic_type_support.auto_fill_type_object(false);
            // WORKAROUND END

            // Register it within the DomainParticipant
            if (pair.second && !dds_participant_->register_type(dynamic_type_support))
            {
                std::ostringstream err;
                err << "Dynamic type '" << type_name << "' registration failed";

                throw ROS2MiddlewareException(logger_, err.str());
            }
        }

        if (pair.second)
        {
            logger_ << utils::Logger::Level::DEBUG
                    << "Registered type '" << type_name << "' in topic '"
                    << topic_name << "'" << std::endl;

            Conversion::register_type(topic_name, &dynamic_type_support);
        }
        else
        {
            logger_ << utils::Logger::Level::WARN
                    << "Failed registering type '" << type_name << "' in topic '"
                    << topic_name << "'" << std::endl;
        }
    }
    else
    {
        std::ostringstream err;
        err << "Dynamic type '" << type_name << "' for topic '"
            << topic_name << "' was not correctly built";

        throw ROS2MiddlewareException(logger_, err.str());
    }
}

fastrtps::types::DynamicData* Participant::create_dynamic_data(
        const std::string& topic_name) const
{
    auto topic_to_type_it = topic_to_type_.find(topic_name);
    if (topic_to_type_.end() == topic_to_type_it)
    {
        std::ostringstream err;
        err << "Creating dynamic data for topic '" << topic_name
            << "' failed because the topic was not registered";

        throw ROS2MiddlewareException(logger_, err.str());
    }

    auto types_it = types_.find(topic_to_type_it->second);
    if (types_.end() == types_it)
    {
        std::ostringstream err;
        err << "Creating dynamic data: dynamic type '" << types_it->first << "' not defined";

        throw ROS2MiddlewareException(logger_, err.str());
    }

    const fastrtps::types::DynamicType_ptr& dynamic_type_ = types_it->second.GetDynamicType();
    return fastrtps::types::DynamicDataFactory::get_instance()->create_data(dynamic_type_);
}

void Participant::delete_dynamic_data(
        fastrtps::types::DynamicData* data) const
{
    DynamicDataFactory::get_instance()->delete_data(data);
}

const fastrtps::types::DynamicType* Participant::get_dynamic_type(
        const std::string& name) const
{
    auto it = types_.find(name);
    if (it == types_.end())
    {
        return nullptr;
    }

    return static_cast<const fastrtps::types::DynamicType*>(it->second.GetDynamicType().get());
}

const std::string& Participant::get_topic_type(
        const std::string& topic) const
{
    return topic_to_type_.at(topic);
}

void Participant::associate_topic_to_dds_entity(
        ::fastdds::dds::Topic* topic,
        ::fastdds::dds::DomainEntity* entity)
{
    std::unique_lock<std::mutex> lock(topic_to_entities_mtx_);

    if (topic_to_entities_.end() == topic_to_entities_.find(topic))
    {
        std::set<::fastdds::dds::DomainEntity*> entities;
        entities.emplace(entity);

        topic_to_entities_[topic] = std::move(entities);
    }
    else
    {
        topic_to_entities_.at(topic).emplace(entity);
    }
}

bool Participant::dissociate_topic_from_dds_entity(
        ::fastdds::dds::Topic* topic,
        ::fastdds::dds::DomainEntity* entity)
{
    // std::unique_lock<std::mutex> lock(topic_to_entities_mtx_);

    if (1 == topic_to_entities_.at(topic).size())
    {
        // Only one entity remains in the map
        topic_to_entities_.erase(topic);
        return true;
    }
    else // Size should not be 0 at this point
    {
        topic_to_entities_.at(topic).erase(entity);
        return false;
    }
}

} //  namespace ros2
} //  namespace sh
} //  namespace is
} //  namespace eprosima
