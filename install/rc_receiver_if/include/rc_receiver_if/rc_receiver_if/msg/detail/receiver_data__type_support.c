// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rc_receiver_if:msg/ReceiverData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rc_receiver_if/msg/detail/receiver_data__rosidl_typesupport_introspection_c.h"
#include "rc_receiver_if/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rc_receiver_if/msg/detail/receiver_data__functions.h"
#include "rc_receiver_if/msg/detail/receiver_data__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__ReceiverData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rc_receiver_if__msg__ReceiverData__init(message_memory);
}

void rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__ReceiverData_fini_function(void * message_memory)
{
  rc_receiver_if__msg__ReceiverData__fini(message_memory);
}

size_t rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__size_function__ReceiverData__channel_data(
  const void * untyped_member)
{
  (void)untyped_member;
  return 18;
}

const void * rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_const_function__ReceiverData__channel_data(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_function__ReceiverData__channel_data(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__fetch_function__ReceiverData__channel_data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_const_function__ReceiverData__channel_data(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__assign_function__ReceiverData__channel_data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_function__ReceiverData__channel_data(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__size_function__ReceiverData__wheel_control(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_const_function__ReceiverData__wheel_control(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_function__ReceiverData__wheel_control(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__fetch_function__ReceiverData__wheel_control(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_const_function__ReceiverData__wheel_control(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__assign_function__ReceiverData__wheel_control(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_function__ReceiverData__wheel_control(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__size_function__ReceiverData__twist(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_const_function__ReceiverData__twist(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_function__ReceiverData__twist(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__fetch_function__ReceiverData__twist(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_const_function__ReceiverData__twist(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__assign_function__ReceiverData__twist(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_function__ReceiverData__twist(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__ReceiverData_message_member_array[3] = {
  {
    "channel_data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    18,  // array size
    false,  // is upper bound
    offsetof(rc_receiver_if__msg__ReceiverData, channel_data),  // bytes offset in struct
    NULL,  // default value
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__size_function__ReceiverData__channel_data,  // size() function pointer
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_const_function__ReceiverData__channel_data,  // get_const(index) function pointer
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_function__ReceiverData__channel_data,  // get(index) function pointer
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__fetch_function__ReceiverData__channel_data,  // fetch(index, &value) function pointer
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__assign_function__ReceiverData__channel_data,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "wheel_control",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(rc_receiver_if__msg__ReceiverData, wheel_control),  // bytes offset in struct
    NULL,  // default value
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__size_function__ReceiverData__wheel_control,  // size() function pointer
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_const_function__ReceiverData__wheel_control,  // get_const(index) function pointer
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_function__ReceiverData__wheel_control,  // get(index) function pointer
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__fetch_function__ReceiverData__wheel_control,  // fetch(index, &value) function pointer
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__assign_function__ReceiverData__wheel_control,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "twist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(rc_receiver_if__msg__ReceiverData, twist),  // bytes offset in struct
    NULL,  // default value
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__size_function__ReceiverData__twist,  // size() function pointer
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_const_function__ReceiverData__twist,  // get_const(index) function pointer
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__get_function__ReceiverData__twist,  // get(index) function pointer
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__fetch_function__ReceiverData__twist,  // fetch(index, &value) function pointer
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__assign_function__ReceiverData__twist,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__ReceiverData_message_members = {
  "rc_receiver_if__msg",  // message namespace
  "ReceiverData",  // message name
  3,  // number of fields
  sizeof(rc_receiver_if__msg__ReceiverData),
  rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__ReceiverData_message_member_array,  // message members
  rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__ReceiverData_init_function,  // function to initialize message memory (memory has to be allocated)
  rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__ReceiverData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__ReceiverData_message_type_support_handle = {
  0,
  &rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__ReceiverData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rc_receiver_if
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rc_receiver_if, msg, ReceiverData)() {
  if (!rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__ReceiverData_message_type_support_handle.typesupport_identifier) {
    rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__ReceiverData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rc_receiver_if__msg__ReceiverData__rosidl_typesupport_introspection_c__ReceiverData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
