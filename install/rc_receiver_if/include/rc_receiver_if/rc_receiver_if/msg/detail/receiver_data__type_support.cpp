// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rc_receiver_if:msg/ReceiverData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rc_receiver_if/msg/detail/receiver_data__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rc_receiver_if
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ReceiverData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rc_receiver_if::msg::ReceiverData(_init);
}

void ReceiverData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rc_receiver_if::msg::ReceiverData *>(message_memory);
  typed_message->~ReceiverData();
}

size_t size_function__ReceiverData__channel_data(const void * untyped_member)
{
  (void)untyped_member;
  return 18;
}

const void * get_const_function__ReceiverData__channel_data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int16_t, 18> *>(untyped_member);
  return &member[index];
}

void * get_function__ReceiverData__channel_data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int16_t, 18> *>(untyped_member);
  return &member[index];
}

void fetch_function__ReceiverData__channel_data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int16_t *>(
    get_const_function__ReceiverData__channel_data(untyped_member, index));
  auto & value = *reinterpret_cast<int16_t *>(untyped_value);
  value = item;
}

void assign_function__ReceiverData__channel_data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int16_t *>(
    get_function__ReceiverData__channel_data(untyped_member, index));
  const auto & value = *reinterpret_cast<const int16_t *>(untyped_value);
  item = value;
}

size_t size_function__ReceiverData__wheel_control(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__ReceiverData__wheel_control(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__ReceiverData__wheel_control(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 2> *>(untyped_member);
  return &member[index];
}

void fetch_function__ReceiverData__wheel_control(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ReceiverData__wheel_control(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ReceiverData__wheel_control(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ReceiverData__wheel_control(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__ReceiverData__twist(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__ReceiverData__twist(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__ReceiverData__twist(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 2> *>(untyped_member);
  return &member[index];
}

void fetch_function__ReceiverData__twist(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ReceiverData__twist(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ReceiverData__twist(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ReceiverData__twist(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ReceiverData_message_member_array[3] = {
  {
    "channel_data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    18,  // array size
    false,  // is upper bound
    offsetof(rc_receiver_if::msg::ReceiverData, channel_data),  // bytes offset in struct
    nullptr,  // default value
    size_function__ReceiverData__channel_data,  // size() function pointer
    get_const_function__ReceiverData__channel_data,  // get_const(index) function pointer
    get_function__ReceiverData__channel_data,  // get(index) function pointer
    fetch_function__ReceiverData__channel_data,  // fetch(index, &value) function pointer
    assign_function__ReceiverData__channel_data,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "wheel_control",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(rc_receiver_if::msg::ReceiverData, wheel_control),  // bytes offset in struct
    nullptr,  // default value
    size_function__ReceiverData__wheel_control,  // size() function pointer
    get_const_function__ReceiverData__wheel_control,  // get_const(index) function pointer
    get_function__ReceiverData__wheel_control,  // get(index) function pointer
    fetch_function__ReceiverData__wheel_control,  // fetch(index, &value) function pointer
    assign_function__ReceiverData__wheel_control,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "twist",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(rc_receiver_if::msg::ReceiverData, twist),  // bytes offset in struct
    nullptr,  // default value
    size_function__ReceiverData__twist,  // size() function pointer
    get_const_function__ReceiverData__twist,  // get_const(index) function pointer
    get_function__ReceiverData__twist,  // get(index) function pointer
    fetch_function__ReceiverData__twist,  // fetch(index, &value) function pointer
    assign_function__ReceiverData__twist,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ReceiverData_message_members = {
  "rc_receiver_if::msg",  // message namespace
  "ReceiverData",  // message name
  3,  // number of fields
  sizeof(rc_receiver_if::msg::ReceiverData),
  ReceiverData_message_member_array,  // message members
  ReceiverData_init_function,  // function to initialize message memory (memory has to be allocated)
  ReceiverData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ReceiverData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ReceiverData_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace rc_receiver_if


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rc_receiver_if::msg::ReceiverData>()
{
  return &::rc_receiver_if::msg::rosidl_typesupport_introspection_cpp::ReceiverData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rc_receiver_if, msg, ReceiverData)() {
  return &::rc_receiver_if::msg::rosidl_typesupport_introspection_cpp::ReceiverData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
