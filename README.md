<a href="https://integration-service.docs.eprosima.com/"><img src="https://github.com/eProsima/Integration-Service/blob/main/docs/images/logo.png?raw=true" hspace="8" vspace="2" height="100" ></a>

# ROS 2 System Handle

[![ROS 2 SH CI Status](https://github.com/eProsima/ROS2-SH/actions/workflows/ci.yml/badge.svg)](https://github.com/eProsima/ROS2-SH/actions)

## Introduction

### What is a System Handle?

A [System Handle](https://integration-service.docs.eprosima.com/en/latest/sh.html) is a plugin that allows a certain middleware
or communication protocol to speak the same language used by the [eProsima Integration Service](https://github.com/eProsima/Integration-Service),
that is, *Extensible and Dynamic Topic Types for DDS* (**xTypes**);
specifically, *Integration Service* bases its intercommunication abilities on eProsima's open source
implementation for the *xTypes* protocol, that is, [eProsima xTypes](https://github.com/eProsima/xtypes).

<p align="center">
  <a href="https://integration-service.docs.eprosima.com/en/latest/sh.html"><img src="docs/images/system-handle-architecture.png"></a>
</p>

### The ROS 2 SystemHandle

<a href="https://docs.ros.org/en/foxy/"><img src="docs/images/ros2_logo.png" align="left" hspace="8" vspace="2" width="120"></a>

This repository contains the source code of *Integration Service* **System Handle**
for the [ROS 2](https://docs.ros.org/en/foxy) middleware protocol, widely used in the robotics field.

This *System Handle* can be used for two main purposes:

1. Connection between a *ROS 2* application and an application running over a different middleware implementation.
  This is the classic use-case approach for *Integration Service*.

1. Connecting two *ROS 2* applications running under different Domain IDs.

## Configuration

*Integration Service* is configured by means of a YAML configuration file, which specifies
the middlewares, topics and/or services involved in the intercommunication process, as well as
their topic/service types and the data exchange flow. This configuration file is loaded during
runtime, so there is no need to recompile any package before switching to a whole new
intercommunication architecture.

To get a more precise idea on how these YAML files have to be filled and which fields they require
in order to succesfully configure and launch an *Integration Service* project, please refer to the
[dedicated configuration section](https://integration-service.docs.eprosima.com/en/latest/yaml_config.html) of the official documentation.

Regarding the *ROS 2 System Handle*, there are several specific parameters which can be configured
for the ROS 2 middleware. All of these parameters are optional, and fall as suboptions of the main
five sections described in the *Configuration* chapter of the *Integration Service* repository:

* `systems`: The system `type` must be `ros2`. In addition to the `type` and `types-from` fields,
  the *ROS 2 System Handle* accepts the following specific configuration fields:

  ```yaml
  systems:
    ros2:
      type: ros2
      namespace: "/"
      node_name: "my_ros2_node"
      domain: 4
  ```
  * `namespace`: The *namespace* of the ROS 2 node created by the *ROS 2 System Handle*.

  * `node_name`: The *ROS 2 System Handle* node name.

  * `domain`: Provides with an easy way to change the *Domain ID* of the ROS 2 entities created
    by the *ROS 2 System Handle*.

## Examples

There are several *Integration Service* examples using the *ROS 2 System Handle* available
in the project's [main source code repository]([https://](https://github.com/eProsima/Integration-Service/tree/main/examples)).

Some of these examples, where the *ROS 2 System Handle* plays a different role in each of them, are introduced here.

<a href="https://integration-service.docs.eprosima.com/en/latest/ros1-ros2.html"><img align="left" width="15" height="38" src="https://via.placeholder.com/15/40c15d/000000?text=+" alt="Green icon"></a>

### ROS 2 - ROS 1 bridge  (publisher -> subscriber)

In this example, *Integration Service* uses both this *ROS 2 System Handle* and the *ROS 1 System Handle*
to transmit data coming from a ROS 2 publisher into the ROS 1 data space, so that it can be
consumed by a ROS 1 subscriber on the same topic, and viceversa.

<p align="center">
  <a href="https://integration-service.docs.eprosima.com/en/latest/ros1-ros2.html"><img src="docs/images/ros2_ros1_pubsub_example.png" width="500"></a>
</p>

The configuration file used by *Integration Service* for this example can be found
[here](https://github.com/eProsima/Integration-Service/blob/main/examples/basic/ros1_ros2__helloworld.yaml).

For a detailed step by step guide on how to build and test this example, please refer to the
[dedicated section](https://integration-service.docs.eprosima.com/en/latest/ros1-ros2.html) in the official documentation.

<a href="https://integration-service.docs.eprosima.com/en/latest/dds-ros2.html"><img align="left" width="15" height="38" src="https://via.placeholder.com/15/40c15d/000000?text=+" alt="Green icon"></a>

### ROS 2 - DDS bridge  (publisher -> subscriber)

In this example, *Integration Service* uses both this *ROS 2 System Handle* and the *Fast DDS System Handle*
to transmit data coming from a ROS 2 publisher into the DDS data space, so that it can be
consumed by a Fast DDS subscriber on the same topic, and viceversa.

<p align="center">
  <a href="https://integration-service.docs.eprosima.com/en/latest/ros1-ros2.html"><img src="docs/images/ros2_dds_pubsub_example.png" width="500"></a>
</p>

The configuration file used by *Integration Service* for this example can be found
[here](https://github.com/eProsima/Integration-Service/blob/main/examples/basic/fastdds_ros2__helloworld.yaml).

For a detailed step by step guide on how to build and test this example, please refer to the
[dedicated section](https://integration-service.docs.eprosima.com/en/latest/dds-ros2.html) in the official documentation.


<!-- TODO: add YAML and applications for DDS and ROS2 to test this
### ROS 2 service server addressing petitions coming from a DDS service client

The configuration file for this example can be found
[here](TODO).

Below, a high level diagram is presented, showing which entities will *Integration Service* create
to forward the petitions requested from a ROS 2 client application to a ROS 2 service server application,
which will process them and produce a reply message which will be transmited back to the DDS client:

![ROS2_server_and_DDS_client](TODO)

For a detailed step by step guide on how to build and test this example, please refer to the
[official documentation](TODO: link).
-->

<a href="https://integration-service.docs.eprosima.com/en/latest/ros2_change_domain.html"><img align="left" width="15" height="38" src="https://via.placeholder.com/15/40c15d/000000?text=+" alt="Green icon"></a>

### ROS 2 Domain ID change

In this example, *Integration Service* uses this *ROS 2 System Handle*
to forward the messages sent from a ROS 2 publisher hosted on a participant with domain ID **5** to
a subscriber created under domain ID **10**.

<p align="center">
  <a href="https://integration-service.docs.eprosima.com/en/latest/ros2_change_domain.html"><img src="docs/images/ros2_domain_id_change.png" width="600"></a>
</p>

The configuration file for this example can be found
[here](https://github.com/eProsima/Integration-Service/blob/main/examples/basic/ros2__domain_id_change.yaml).

For a detailed step by step guide on how to build and test this example, please refer to the
[dedicated section](https://integration-service.docs.eprosima.com/en/latest/ros2_change_domain.html) in the official documentation.


## Compilation flags

Besides the [global compilation flags](<!-- TODO: link to IS readme section-->) available for the
whole *Integration Service* product suite, there are some specific flags which apply only to the
*ROS 2 System Handle*; they are listed below:

* `BUILD_ROS2_TESTS`: Allows to specifically compile the *ROS 2 System Handle* unitary and
  integration tests; this is useful to avoid compiling each *System Handle's* test suite present
  in the `colcon` workspace, which is what would happen if using the `BUILD_TESTS` flag; and thus,
  minimizing the building time; to use it, after making sure that the *ROS 2 System Handle*
  is present in the `colcon` workspace, the following command must be executed:
  ```bash
  ~/is_ws$ colcon build --cmake-args -DBUILD_ROS2_TESTS=ON
  ```

* `MIX_ROS_PACKAGES`: It accepts as an argument a list of [ROS 2 packages](https://index.ros.org/packages/),
  such as `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`... for which the required transformation
  library to convert the specific ROS 2 type definitions into *xTypes*, and the other way around, will be built.
  This library is also known within the *Integration Service* context as `Middleware Interface Extension`
  or `mix` library.

  By default, only the `std_msgs_mix` library is compiled, unless the `BUILD_TESTS`
  or `BUILD_ROS2_TESTS` is used, case in which some additional ROS 2 packages `mix` files
  required for testing will be built.

  If the user wants to compile some additional packages to use them with *Integration Service*,
  the following command must be launched to compile it, adding as many packages to the list as desired:
  ```bash
  ~/is_ws$ colcon build --cmake-args -DMIX_ROS_PACKAGES="std_msgs geometry_msgs sensor_msgs nav_msgs"
  ```

<!-- TODO: complete when it is uploaded to read the docs
## API Reference
-->

## License

This repository is open-sourced under the *Apache-2.0* license. See the [LICENSE](LICENSE) file for more details.

## Getting help

If you need support you can reach us by mail at `support@eProsima.com` or by phone at `+34 91 804 34 48`.
