// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_if:srv/MotorPos.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_IF__SRV__DETAIL__MOTOR_POS__STRUCT_H_
#define MOTOR_IF__SRV__DETAIL__MOTOR_POS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/MotorPos in the package motor_if.
typedef struct motor_if__srv__MotorPos_Request
{
  float linear;
  float angular;
} motor_if__srv__MotorPos_Request;

// Struct for a sequence of motor_if__srv__MotorPos_Request.
typedef struct motor_if__srv__MotorPos_Request__Sequence
{
  motor_if__srv__MotorPos_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_if__srv__MotorPos_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MotorPos in the package motor_if.
typedef struct motor_if__srv__MotorPos_Response
{
  int32_t result;
} motor_if__srv__MotorPos_Response;

// Struct for a sequence of motor_if__srv__MotorPos_Response.
typedef struct motor_if__srv__MotorPos_Response__Sequence
{
  motor_if__srv__MotorPos_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_if__srv__MotorPos_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_IF__SRV__DETAIL__MOTOR_POS__STRUCT_H_
