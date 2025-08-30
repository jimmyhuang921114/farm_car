// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rc_receiver_if:msg/ReceiverData.idl
// generated code does not contain a copyright notice

#ifndef RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__STRUCT_H_
#define RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ReceiverData in the package rc_receiver_if.
typedef struct rc_receiver_if__msg__ReceiverData
{
  int16_t channel_data[18];
  float wheel_control[2];
  float twist[2];
} rc_receiver_if__msg__ReceiverData;

// Struct for a sequence of rc_receiver_if__msg__ReceiverData.
typedef struct rc_receiver_if__msg__ReceiverData__Sequence
{
  rc_receiver_if__msg__ReceiverData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rc_receiver_if__msg__ReceiverData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__STRUCT_H_
