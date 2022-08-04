from pathlib import Path
from types import NoneType
from typing import Any, List, Type

from ament_index_python.packages import get_package_share_directory

from rosidl_parser.parser import parse_idl_file
import rosidl_parser.definition as idl_def
import rosidl_adapter.parser as rosidl_parser
from rosidl_adapter.msg import MSG_TYPE_TO_IDL

IDL_TYPE_TO_MSG = {v: k for k, v in MSG_TYPE_TO_IDL.items()}

BOUNDED_TYPES = [idl_def.BoundedSequence, idl_def.Array]
BRACKET_TYPES = BOUNDED_TYPES + [idl_def.UnboundedSequence, idl_def.Array]
STRING_BOUND_TYPES = [idl_def.BoundedString, idl_def.BoundedWString]


def resolve_typename(member_type: Type[idl_def.AbstractType]) -> str:
    if isinstance(member_type, idl_def.BasicType):
        return member_type.typename
    elif isinstance(member_type, idl_def.AbstractWString):
        return "wstring"
    elif isinstance(member_type, idl_def.AbstractString):
        return "string"
    elif isinstance(member_type, idl_def.NamedType):
        return member_type.name
    elif isinstance(member_type, idl_def.AbstractNestedType):
        return resolve_typename(member_type.value_type)
    else:
        return member_type.name


def build_type_string(member_type: Type[idl_def.AbstractType]) -> str:
    type_string = resolve_typename(member_type)

    if type_string in IDL_TYPE_TO_MSG:
        type_string = IDL_TYPE_TO_MSG[type_string]

    has_string_bounds = any(isinstance(member_type, t) for t in STRING_BOUND_TYPES)
    has_brackets = any(isinstance(member_type, t) for t in BRACKET_TYPES)

    if has_string_bounds:
        type_string += f"{rosidl_parser.STRING_UPPER_BOUND_TOKEN}{member_type.string_upper_bound}"

    if has_brackets:
        bounds = ''
        if isinstance(member_type, idl_def.BoundedSequence):
            bounds = f"{rosidl_parser.ARRAY_UPPER_BOUND_TOKEN}{member_type.maximum_size}"
        if isinstance(member_type, idl_def.Array):
            bounds = f"{member_type.size}"
        type_string += f"[{bounds}]"

    return type_string


def find_annotation_value(annotations: List, name: str, value_key="value") -> Any:
    for a in annotations:
        if a.name == name:
            return a.value[value_key]

    return None


def process_constants(constants: List[idl_def.Constant]) -> List[rosidl_parser.Constant]:
    out_constants = []
    for c in constants:
        typename = IDL_TYPE_TO_MSG[c.type.typename]
        out_constants.append(rosidl_parser.Constant(typename, c.name, c.value))
    return out_constants


def process_members(members: List) -> List[rosidl_parser.Field]:
    fields = []
    for m in members:
        type_pkg_name = None
        type_string = build_type_string(m.type)

        if isinstance(m.type, idl_def.NamespacedType):
            type_pkg_name = m.type.namespaces[0]

        field_type = rosidl_parser.Type(type_string, type_pkg_name)

        default_value_str = find_annotation_value(m.annotations, 'default')
        if type(default_value_str) not in [NoneType, str]:
            default_value_str = str(default_value_str)

        fields.append(rosidl_parser.Field(field_type, m.name, default_value_str))

    return fields


def parse_idl_message(pkg_name: str, msg_name: str, idl_msg: idl_def.Message) -> rosidl_parser.MessageSpecification:
    fields = process_members(idl_msg.structure.members)
    constants = process_constants(idl_msg.constants)

    msg = rosidl_parser.MessageSpecification(pkg_name, msg_name, fields, constants)
    # msg.annotations['comment'] = message_comments

    return msg


def parse_idl_to_message_spec(pkg_name: str, interface_file_path: str) -> rosidl_parser.MessageSpecification:
    path = Path(interface_file_path)
    share_dir = get_package_share_directory(pkg_name)
    msg_name = path.stem

    idl_file = parse_idl_file(idl_def.IdlLocator(share_dir, path.relative_to(share_dir)))
    idl_msg = idl_file.content.get_elements_of_type(idl_def.Message)[0]
    return parse_idl_message(pkg_name, msg_name, idl_msg)


def parse_idl_to_service_spec(pkg_name: str, interface_file_path: str) -> rosidl_parser.ServiceSpecification:
    path = Path(interface_file_path)
    share_dir = get_package_share_directory(pkg_name)
    srv_name = path.stem

    idl_file = parse_idl_file(idl_def.IdlLocator(share_dir, path.relative_to(share_dir)))
    idl_srv = idl_file.content.get_elements_of_type(idl_def.Service)[0]
    request_message = parse_idl_message(pkg_name, srv_name + rosidl_parser.SERVICE_REQUEST_MESSAGE_SUFFIX, idl_srv.request_message)
    response_message = parse_idl_message(pkg_name, srv_name + rosidl_parser.SERVICE_RESPONSE_MESSAGE_SUFFIX, idl_srv.response_message)

    return rosidl_parser.ServiceSpecification(pkg_name, srv_name, request_message, response_message)
