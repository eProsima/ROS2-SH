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

#ifndef _IS_SH_ROS2_DYNAMIC__INTERNAL__ROS2MIDDLEWAREEXCEPTION_HPP_
#define _IS_SH_ROS2_DYNAMIC__INTERNAL__ROS2MIDDLEWAREEXCEPTION_HPP_

#include <stdexcept>

#include <is/utils/Log.hpp>

namespace eprosima {
namespace is {
namespace sh {
namespace ros2 {

/**
 * @class ROS2MiddlewareException
 *        Launches a runtime error every time an unexpected behavior occurs
 *        related to *ROS 2 Dynamic* middleware, when configuring or using this is::SystemHandle.
 */
class ROS2MiddlewareException : public std::runtime_error
{
public:

    /**
     * @brief Construct a new ROS2MiddlewareException object.
     *
     * @param[in] logger The logging tool.
     *
     * @param[in] message The message to throw the runtime error with.
     */
    ROS2MiddlewareException(
            const utils::Logger& logger,
            const std::string& message)
        : std::runtime_error(message)
        , from_logger(logger)
    {
    }

    utils::Logger from_logger;
};

} //  namespace ros2
} //  namespace sh
} //  namespace is
} //  namespace eprosima

#endif //  _IS_SH_ROS2_DYNAMIC__INTERNAL__ROS2MIDDLEWAREEXCEPTION_HPP_
