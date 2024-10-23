// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamixel_sdk_custom_interfaces:msg/SetVelocityDual.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY_DUAL__STRUCT_H_
#define DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY_DUAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SetVelocityDual in the package dynamixel_sdk_custom_interfaces.
/**
  * Messages
 */
typedef struct dynamixel_sdk_custom_interfaces__msg__SetVelocityDual
{
  int32_t motorspeed1;
  int32_t motorspeed2;
} dynamixel_sdk_custom_interfaces__msg__SetVelocityDual;

// Struct for a sequence of dynamixel_sdk_custom_interfaces__msg__SetVelocityDual.
typedef struct dynamixel_sdk_custom_interfaces__msg__SetVelocityDual__Sequence
{
  dynamixel_sdk_custom_interfaces__msg__SetVelocityDual * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_sdk_custom_interfaces__msg__SetVelocityDual__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY_DUAL__STRUCT_H_
