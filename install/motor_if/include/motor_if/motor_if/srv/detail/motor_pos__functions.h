// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from motor_if:srv/MotorPos.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_IF__SRV__DETAIL__MOTOR_POS__FUNCTIONS_H_
#define MOTOR_IF__SRV__DETAIL__MOTOR_POS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "motor_if/msg/rosidl_generator_c__visibility_control.h"

#include "motor_if/srv/detail/motor_pos__struct.h"

/// Initialize srv/MotorPos message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * motor_if__srv__MotorPos_Request
 * )) before or use
 * motor_if__srv__MotorPos_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
bool
motor_if__srv__MotorPos_Request__init(motor_if__srv__MotorPos_Request * msg);

/// Finalize srv/MotorPos message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
void
motor_if__srv__MotorPos_Request__fini(motor_if__srv__MotorPos_Request * msg);

/// Create srv/MotorPos message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * motor_if__srv__MotorPos_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
motor_if__srv__MotorPos_Request *
motor_if__srv__MotorPos_Request__create();

/// Destroy srv/MotorPos message.
/**
 * It calls
 * motor_if__srv__MotorPos_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
void
motor_if__srv__MotorPos_Request__destroy(motor_if__srv__MotorPos_Request * msg);

/// Check for srv/MotorPos message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
bool
motor_if__srv__MotorPos_Request__are_equal(const motor_if__srv__MotorPos_Request * lhs, const motor_if__srv__MotorPos_Request * rhs);

/// Copy a srv/MotorPos message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
bool
motor_if__srv__MotorPos_Request__copy(
  const motor_if__srv__MotorPos_Request * input,
  motor_if__srv__MotorPos_Request * output);

/// Initialize array of srv/MotorPos messages.
/**
 * It allocates the memory for the number of elements and calls
 * motor_if__srv__MotorPos_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
bool
motor_if__srv__MotorPos_Request__Sequence__init(motor_if__srv__MotorPos_Request__Sequence * array, size_t size);

/// Finalize array of srv/MotorPos messages.
/**
 * It calls
 * motor_if__srv__MotorPos_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
void
motor_if__srv__MotorPos_Request__Sequence__fini(motor_if__srv__MotorPos_Request__Sequence * array);

/// Create array of srv/MotorPos messages.
/**
 * It allocates the memory for the array and calls
 * motor_if__srv__MotorPos_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
motor_if__srv__MotorPos_Request__Sequence *
motor_if__srv__MotorPos_Request__Sequence__create(size_t size);

/// Destroy array of srv/MotorPos messages.
/**
 * It calls
 * motor_if__srv__MotorPos_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
void
motor_if__srv__MotorPos_Request__Sequence__destroy(motor_if__srv__MotorPos_Request__Sequence * array);

/// Check for srv/MotorPos message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
bool
motor_if__srv__MotorPos_Request__Sequence__are_equal(const motor_if__srv__MotorPos_Request__Sequence * lhs, const motor_if__srv__MotorPos_Request__Sequence * rhs);

/// Copy an array of srv/MotorPos messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
bool
motor_if__srv__MotorPos_Request__Sequence__copy(
  const motor_if__srv__MotorPos_Request__Sequence * input,
  motor_if__srv__MotorPos_Request__Sequence * output);

/// Initialize srv/MotorPos message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * motor_if__srv__MotorPos_Response
 * )) before or use
 * motor_if__srv__MotorPos_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
bool
motor_if__srv__MotorPos_Response__init(motor_if__srv__MotorPos_Response * msg);

/// Finalize srv/MotorPos message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
void
motor_if__srv__MotorPos_Response__fini(motor_if__srv__MotorPos_Response * msg);

/// Create srv/MotorPos message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * motor_if__srv__MotorPos_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
motor_if__srv__MotorPos_Response *
motor_if__srv__MotorPos_Response__create();

/// Destroy srv/MotorPos message.
/**
 * It calls
 * motor_if__srv__MotorPos_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
void
motor_if__srv__MotorPos_Response__destroy(motor_if__srv__MotorPos_Response * msg);

/// Check for srv/MotorPos message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
bool
motor_if__srv__MotorPos_Response__are_equal(const motor_if__srv__MotorPos_Response * lhs, const motor_if__srv__MotorPos_Response * rhs);

/// Copy a srv/MotorPos message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
bool
motor_if__srv__MotorPos_Response__copy(
  const motor_if__srv__MotorPos_Response * input,
  motor_if__srv__MotorPos_Response * output);

/// Initialize array of srv/MotorPos messages.
/**
 * It allocates the memory for the number of elements and calls
 * motor_if__srv__MotorPos_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
bool
motor_if__srv__MotorPos_Response__Sequence__init(motor_if__srv__MotorPos_Response__Sequence * array, size_t size);

/// Finalize array of srv/MotorPos messages.
/**
 * It calls
 * motor_if__srv__MotorPos_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
void
motor_if__srv__MotorPos_Response__Sequence__fini(motor_if__srv__MotorPos_Response__Sequence * array);

/// Create array of srv/MotorPos messages.
/**
 * It allocates the memory for the array and calls
 * motor_if__srv__MotorPos_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
motor_if__srv__MotorPos_Response__Sequence *
motor_if__srv__MotorPos_Response__Sequence__create(size_t size);

/// Destroy array of srv/MotorPos messages.
/**
 * It calls
 * motor_if__srv__MotorPos_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
void
motor_if__srv__MotorPos_Response__Sequence__destroy(motor_if__srv__MotorPos_Response__Sequence * array);

/// Check for srv/MotorPos message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
bool
motor_if__srv__MotorPos_Response__Sequence__are_equal(const motor_if__srv__MotorPos_Response__Sequence * lhs, const motor_if__srv__MotorPos_Response__Sequence * rhs);

/// Copy an array of srv/MotorPos messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_if
bool
motor_if__srv__MotorPos_Response__Sequence__copy(
  const motor_if__srv__MotorPos_Response__Sequence * input,
  motor_if__srv__MotorPos_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_IF__SRV__DETAIL__MOTOR_POS__FUNCTIONS_H_
