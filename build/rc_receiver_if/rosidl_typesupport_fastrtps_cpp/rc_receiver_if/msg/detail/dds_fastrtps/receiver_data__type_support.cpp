// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rc_receiver_if:msg/ReceiverData.idl
// generated code does not contain a copyright notice
#include "rc_receiver_if/msg/detail/receiver_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rc_receiver_if/msg/detail/receiver_data__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

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
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: channel_data
  {
    cdr << ros_message.channel_data;
  }
  // Member: wheel_control
  {
    cdr << ros_message.wheel_control;
  }
  // Member: twist
  {
    cdr << ros_message.twist;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rc_receiver_if
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rc_receiver_if::msg::ReceiverData & ros_message)
{
  // Member: channel_data
  {
    cdr >> ros_message.channel_data;
  }

  // Member: wheel_control
  {
    cdr >> ros_message.wheel_control;
  }

  // Member: twist
  {
    cdr >> ros_message.twist;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rc_receiver_if
get_serialized_size(
  const rc_receiver_if::msg::ReceiverData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: channel_data
  {
    size_t array_size = 18;
    size_t item_size = sizeof(ros_message.channel_data[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: wheel_control
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.wheel_control[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: twist
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.twist[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rc_receiver_if
max_serialized_size_ReceiverData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: channel_data
  {
    size_t array_size = 18;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: wheel_control
  {
    size_t array_size = 2;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: twist
  {
    size_t array_size = 2;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rc_receiver_if::msg::ReceiverData;
    is_plain =
      (
      offsetof(DataType, twist) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ReceiverData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rc_receiver_if::msg::ReceiverData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ReceiverData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rc_receiver_if::msg::ReceiverData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ReceiverData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rc_receiver_if::msg::ReceiverData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ReceiverData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ReceiverData(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ReceiverData__callbacks = {
  "rc_receiver_if::msg",
  "ReceiverData",
  _ReceiverData__cdr_serialize,
  _ReceiverData__cdr_deserialize,
  _ReceiverData__get_serialized_size,
  _ReceiverData__max_serialized_size
};

static rosidl_message_type_support_t _ReceiverData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ReceiverData__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rc_receiver_if

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rc_receiver_if
const rosidl_message_type_support_t *
get_message_type_support_handle<rc_receiver_if::msg::ReceiverData>()
{
  return &rc_receiver_if::msg::typesupport_fastrtps_cpp::_ReceiverData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rc_receiver_if, msg, ReceiverData)() {
  return &rc_receiver_if::msg::typesupport_fastrtps_cpp::_ReceiverData__handle;
}

#ifdef __cplusplus
}
#endif
