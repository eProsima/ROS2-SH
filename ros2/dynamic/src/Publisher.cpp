/*
 * Copyright 2019 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <is/sh/ros2/Publisher.hpp>
#include <is/sh/ros2/Conversion.hpp>

#include <fastdds/dds/publisher/PublisherListener.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>

#include <iostream>
#include <sstream>

#define NS_TO_S(nanoseconds) (nanoseconds / (1000 * 1000 * 1000))

namespace eprosima {
namespace is {
namespace sh {
namespace ros2 {

Publisher::Publisher(
        Participant* participant,
        const std::string& topic_name,
        const xtypes::DynamicType& message_type,
        const YAML::Node& config)
    : participant_(participant)
    , topic_name_(topic_name)
    , logger_("is::sh::ROS2_Dynamic::Publisher")
{

    // Adds the type name mangling
    std::string type_name = message_type.name();
    std::size_t found = type_name.find_last_of("::");

    // packagename::msg::dds_::typename_
    if (found == std::string::npos)
    {
        logger_ << utils::Logger::Level::ERROR
                << "The type must follow the ROS2 naming convention" << std::endl;
        return;
    }
    else
    {
        type_name.insert(found + 1, "dds_::");
        type_name.append("_");
    }

    fastrtps::types::DynamicTypeBuilder* builder = Conversion::create_builder(message_type, type_name);

    if (builder != nullptr)
    {
        participant->register_dynamic_type(topic_name, type_name, builder);
    }
    else

    {
        throw ROS2MiddlewareException(
                  logger_, "Cannot create builder for type " + type_name);
    }

    dynamic_data_ = participant->create_dynamic_data(topic_name);

    // Retrieve DDS participant
    ::fastdds::dds::DomainParticipant* dds_participant = participant->get_dds_participant();
    if (!dds_participant)
    {
        throw ROS2MiddlewareException(
                  logger_, "Trying to create a publisher without a DDS participant!");
    }

    // Create DDS publisher with default publisher QoS
    dds_publisher_ = dds_participant->create_publisher(::fastdds::dds::PUBLISHER_QOS_DEFAULT);
    if (dds_publisher_)
    {
        logger_ << utils::Logger::Level::DEBUG
                << "Created ROS2 Dynamic publisher for topic '" << topic_name << "'" << std::endl;
    }
    else
    {
        std::ostringstream err;
        err << "ROS2 Dynamic publisher for topic '" << topic_name << "' was not created";

        throw ROS2MiddlewareException(logger_, err.str());
    }

    // Create DDS topic
    auto topic_description = dds_participant->lookup_topicdescription(topic_name);
    if (!topic_description)
    {
        dds_topic_ = dds_participant->create_topic(
            topic_name, type_name, ::fastdds::dds::TOPIC_QOS_DEFAULT);
        if (dds_topic_)
        {
            logger_ << utils::Logger::Level::DEBUG
                    << "Created ROS2 Dynamic topic '" << topic_name << "' with type '"
                    << type_name << "'" << std::endl;
        }
        else
        {
            std::ostringstream err;
            err << "ROS2 Dynamic topic '" << topic_name << "' with type '"
                << type_name << "' was not created";

            throw ROS2MiddlewareException(logger_, err.str());
        }
    }
    else
    {
        dds_topic_ = static_cast<::fastdds::dds::Topic*>(topic_description);
    }

    // Create DDS datawriter
    ::fastdds::dds::DataWriterQos datawriter_qos = ::fastdds::dds::DATAWRITER_QOS_DEFAULT;

    // Configure datawriter QoS according to config node
    if (config["qos"])
    {
        get_qos_from_config(&datawriter_qos, config["qos"]);
    }

    dds_datawriter_ = dds_publisher_->create_datawriter(dds_topic_, datawriter_qos, this);
    if (dds_datawriter_)
    {
        logger_ << utils::Logger::Level::DEBUG
                << "Created ROS2 Dynamic datawriter for topic '" << topic_name << "'" << std::endl;
        participant_->associate_topic_to_dds_entity(dds_topic_, dds_datawriter_);
    }
    else
    {
        std::ostringstream err;
        err << "ROS2 Dynamic datawriter for topic '" << topic_name << "' was not created";

        throw ROS2MiddlewareException(logger_, err.str());
    }
}

Publisher::~Publisher()
{
    std::unique_lock<std::mutex> lock(data_mtx_);
    participant_->delete_dynamic_data(dynamic_data_);

    bool delete_topic = participant_->dissociate_topic_from_dds_entity(dds_topic_, dds_datawriter_);

    dds_datawriter_->set_listener(nullptr);
    dds_publisher_->delete_datawriter(dds_datawriter_);
    participant_->get_dds_participant()->delete_publisher(dds_publisher_);

    if (delete_topic)
    {
        participant_->get_dds_participant()->delete_topic(dds_topic_);
    }
}

bool Publisher::publish(
        const ::xtypes::DynamicData& message)
{
    std::unique_lock<std::mutex> lock(data_mtx_);

    logger_ << utils::Logger::Level::INFO
            << "Sending message from Integration Service to ROS 2 for topic '" << topic_name_ << "': "
            << "[[ " << message << " ]]" << std::endl;

    bool success = Conversion::xtypes_to_fastdds(message, dynamic_data_);
    if (success)
    {
        success = dds_datawriter_->write(static_cast<void*>(dynamic_data_));
    }
    else
    {
        logger_ << utils::Logger::Level::ERROR
                << "Failed to convert message from Integration Service to ROS 2 for topic '"
                << topic_name_ << "': [[ " << message << " ]]" << std::endl;
    }

    return success;
}

const std::string& Publisher::topic_name() const
{
    return topic_name_;
}

const fastrtps::rtps::InstanceHandle_t Publisher::get_dds_instance_handle() const
{
    return dds_datawriter_->get_instance_handle();
}

void Publisher::on_publication_matched(
        ::fastdds::dds::DataWriter* writer,
        const ::fastdds::dds::PublicationMatchedStatus& info)
{
    if (1 == info.current_count_change)
    {
        logger_ << utils::Logger::Level::INFO
                << "Publisher for topic '" << topic_name_ << "' matched in domain "
                << writer->get_publisher()->get_participant()->get_domain_id() << std::endl;
    }
    else if (-1 == info.current_count_change)
    {
        logger_ << utils::Logger::Level::INFO
                << "Publisher for topic '" << topic_name_ << "' unmatched" << std::endl;
    }
}

void Publisher::get_qos_from_config(
    ::fastdds::dds::DataWriterQos* dw_qos,
    const YAML::Node& config)
{
    logger_ << utils::Logger::Level::DEBUG
                    << "Publisher created with QoS: " << YAML::Dump(config) << std::endl;

    // Configure datawriter QoS according to config node
    if (config["history"])
    {
        ::fastdds::dds::HistoryQosPolicy hist_policy;
        if (config["history"]["kind"])
        {
            std::string hist_kind = config["history"]["kind"].as<std::string>();
            if (hist_kind == "KEEP_LAST")
            {
                hist_policy.kind = ::fastdds::dds::KEEP_LAST_HISTORY_QOS;
            }
            else if (hist_kind == "KEEP_ALL")
            {
                hist_policy.kind = ::fastdds::dds::KEEP_ALL_HISTORY_QOS;
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
            hist_policy.depth = config["history"]["depth"].as<int>();
        }

        dw_qos->history(hist_policy);
    }

    if (config["reliability"])
    {
        ::fastdds::dds::ReliabilityQosPolicy rel_policy;
        std::string rel_kind = config["reliability"].as<std::string>();
        if (rel_kind == "RELIABLE")
        {
            rel_policy.kind = ::fastdds::dds::RELIABLE_RELIABILITY_QOS;
        }
        else if (rel_kind == "BEST_EFFORT")
        {
            rel_policy.kind = ::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
        }
        else
        {
            logger_ << utils::Logger::Level::WARN
                    << "Reliability QoS kind is unknown. "
                    << "The valid values are: RELIABLE and BEST_EFFORT." << std::endl;
        }

        dw_qos->reliability(rel_policy);
    }

    if (config["durability"])
    {
        ::fastdds::dds::DurabilityQosPolicy dur_policy;
        std::string dur_kind = config["durability"].as<std::string>();
        if (dur_kind == "VOLATILE")
        {
            dur_policy.kind = ::fastdds::dds::VOLATILE_DURABILITY_QOS;
        }
        else if (dur_kind == "TRANSIENT_LOCAL")
        {
            dur_policy.kind = ::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
        }
        else
        {
            logger_ << utils::Logger::Level::WARN
                    << "Durability QoS kind is unknown. "
                    << "The valid values are: VOLATILE and TRANSIENT_LOCAL." << std::endl;
        }

        dw_qos->durability(dur_policy);
    }

    if (config["deadline"])
    {
        ::fastdds::dds::DeadlineQosPolicy d_policy;
        eprosima::fastrtps::rtps::Time_t d_period = eprosima::fastrtps::c_TimeInfinite;
        if (config["deadline"]["sec"])
        {
            d_period.seconds(config["deadline"]["sec"].as<int32_t>());
        }

        if (config["deadline"]["nanosec"])
        {
            d_period.nanosec(config["deadline"]["nanosec"].as<uint32_t>());
        }

        d_policy.period = d_period.to_duration_t();
        dw_qos->deadline(d_policy);
    }

    if (config["lifespan"])
    {
        ::fastdds::dds::LifespanQosPolicy life_policy;
        eprosima::fastrtps::rtps::Time_t life_duration = eprosima::fastrtps::c_TimeInfinite;
        if (config["lifespan"]["sec"])
        {
            life_duration.seconds(config["lifespan"]["sec"].as<int32_t>());
        }

        if (config["lifespan"]["nanosec"])
        {
            life_duration.nanosec(config["lifespan"]["nanosec"].as<uint32_t>());
        }

        life_policy.duration = life_duration.to_duration_t();
        dw_qos->lifespan(life_policy);
    }

    if (config["liveliness"])
    {
        ::fastdds::dds::LivelinessQosPolicy live_policy;
        if (config["liveliness"]["kind"])
        {
            std::string live_kind = config["liveliness"]["kind"].as<std::string>();
            if (live_kind == "AUTOMATIC")
            {
                live_policy.kind = ::fastdds::dds::AUTOMATIC_LIVELINESS_QOS;
            }
            else if (live_kind == "MANUAL_BY_TOPIC")
            {
                live_policy.kind = ::fastdds::dds::MANUAL_BY_TOPIC_LIVELINESS_QOS;
            }
            else
            {
                logger_ << utils::Logger::Level::WARN
                        << "Liveliness QoS kind is unknown. "
                        << "The valid values are: AUTOMATIC and MANUAL_BY_TOPIC." << std::endl;
            }
        }

        eprosima::fastrtps::rtps::Time_t live_duration = eprosima::fastrtps::c_TimeInfinite;
        bool lease_duration_changed = false;
        if (config["liveliness"]["sec"])
        {
            live_duration.seconds(config["liveliness"]["sec"].as<int32_t>());
            lease_duration_changed = true;
        }

        if (config["liveliness"]["nanosec"])
        {
            live_duration.nanosec(config["liveliness"]["nanosec"].as<uint32_t>());
            lease_duration_changed = true;
        }

        if (lease_duration_changed)
        {
            double ns = live_duration.to_ns() * 2.0 / 3.0;
            double s = NS_TO_S(ns);
            live_policy.announcement_period = eprosima::fastrtps::Duration_t(s);
        }
        live_policy.lease_duration = live_duration.to_duration_t();

        dw_qos->liveliness(live_policy);
    }
}

} //  namespace ros2
} //  namespace sh
} //  namespace is
} //  namespace eprosima
