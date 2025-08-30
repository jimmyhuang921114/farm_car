// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from rc_receiver_if:msg/ReceiverData.idl
// generated code does not contain a copyright notice

#ifndef RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "rc_receiver_if/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "rc_receiver_if/msg/detail/receiver_data__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace rc_receiver_if
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rc_receiver_if
cdr_serialize(
  const rc_receiver_if::msg::ReceiverData & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rc_receiver_if
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rc_receiver_if::msg::ReceiverData & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rc_receiver_if
get_serialized_size(
  const rc_receiver_if::msg::ReceiverData & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rc_receiver_if
max_serialized_size_ReceiverData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rc_receiver_if

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rc_receiver_if
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rc_receiver_if, msg, ReceiverData)();

#ifdef __cplusplus
}
#endif

#endif  // RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
