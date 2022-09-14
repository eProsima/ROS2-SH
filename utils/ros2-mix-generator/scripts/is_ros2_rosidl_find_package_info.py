#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
from pathlib import Path

try:
    from rosidl_adapter.parser import parse_message_file
    from rosidl_adapter.parser import parse_service_file
    from rosidl_parser.parser import parse_idl_file
    from rosidl_parser.definition import IdlLocator, NamespacedType, Message, Service

except ImportError:
    print('Unable to import rosidl_adapter. Please source a ROS2 installation first.', end='', file=sys.stderr)
    sys.exit(1)

from ament_index_python.packages import get_package_share_directory


class PackageInfo:

    def __init__(self, name):
        self.pkg_name = name
        self.msg_files = []
        self.srv_files = []
        self.dependencies = []


def find_package_info(requested_pkg_name):
    info = PackageInfo(requested_pkg_name)

    share_dir = get_package_share_directory(requested_pkg_name)

    message_dir = Path(share_dir) / 'msg'
    if message_dir.exists():
        for msg_file in message_dir.glob("*.msg"):
            info.msg_files.append(msg_file)

            msg = parse_message_file(requested_pkg_name, msg_file)
            for field in msg.fields:
                if not field.type.is_primitive_type():
                    info.dependencies.append(field.type.pkg_name)

        ignore_msgs = {p.stem for p in info.msg_files}

        for msg_file in message_dir.glob("*.idl"):
            if msg_file.stem in ignore_msgs:
                continue

            info.msg_files.append(msg_file)

            idl_file = parse_idl_file(IdlLocator(share_dir, msg_file.relative_to(share_dir)))
            idl_msg = idl_file.content.get_elements_of_type(Message)[0]
            for member in idl_msg.structure.members:
                if isinstance(member.type, NamespacedType):
                    info.dependencies.append(member.type.namespaces[0])

    service_dir = Path(share_dir) / 'srv'
    if service_dir.exists():
        for srv_file in service_dir.glob("*.srv"):
            info.srv_files.append(srv_file)

            srv = parse_service_file(requested_pkg_name, srv_file)
            for component in [srv.request, srv.response]:
                for field in component.fields:
                    if field.type.is_primitive_type():
                        continue

                    info.dependencies.append(field.type.pkg_name)

        ignore_srvs = {p.stem for p in info.srv_files}

        for srv_file in service_dir.glob("*.idl"):
            if srv_file.stem in ignore_srvs:
                continue

            info.srv_files.append(srv_file)

            idl_file = parse_idl_file(IdlLocator(share_dir, srv_file.relative_to(share_dir)))
            idl_srv = idl_file.content.get_elements_of_type(Service)[0]
            for members in [idl_srv.request_message.structure.members, idl_srv.response_message.structure.members]:
                for member in members:
                    if isinstance(member.type, NamespacedType):
                        info.dependencies.append(member.type.namespaces[0])

    return info


def traverse_packages(root_pkg_name):
    package_queue = [root_pkg_name]
    inspected_packages = {}

    while package_queue:
        next_pkg = package_queue.pop()
        if next_pkg in inspected_packages:
            continue

        info = find_package_info(next_pkg)
        inspected_packages[next_pkg] = info
        for dependency in info.dependencies:
            package_queue.append(dependency)

    return inspected_packages


def print_package_info(root_pkg_name, pkg_info_dict):
    dependency_list = set(pkg_info_dict.keys())
    dependency_list.remove(root_pkg_name)
    dependency_list_str = '#'.join(dependency_list)

    message_files = pkg_info_dict[root_pkg_name].msg_files
    message_files_str = '#'.join(str(f) for f in message_files)

    service_files = pkg_info_dict[root_pkg_name].srv_files
    service_files_str = '#'.join(str(f) for f in service_files)

    file_dependencies = []
    for pkg, info in pkg_info_dict.items():
        file_dependencies.extend(info.msg_files)
        if pkg == root_pkg_name:
            file_dependencies.extend(info.srv_files)

    file_dependencies_str = '#'.join(str(f) for f in file_dependencies)

    output_str = ';'.join([dependency_list_str, message_files_str, service_files_str, file_dependencies_str])
    print(output_str)


def main(cli_args):
    parser = argparse.ArgumentParser(
      description='Find dependencies for a message package.\n'
                  'First line of stdout contains a semi-colon separated list of package names.\n'
                  'Second line of stdout contains a semi-colon separated list of message file locations.')
    parser.add_argument('package', help='The packages whose dependencies should be searched for.')
    args = parser.parse_args(cli_args[1:])

    root_package_name = args.package
    print_package_info(root_package_name, traverse_packages(root_package_name))


if __name__ == '__main__':
    main(sys.argv)
