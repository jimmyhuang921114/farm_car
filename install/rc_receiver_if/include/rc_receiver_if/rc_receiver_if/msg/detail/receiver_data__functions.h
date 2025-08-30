// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rc_receiver_if:msg/ReceiverData.idl
// generated code does not contain a copyright notice

#ifndef RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__FUNCTIONS_H_
#define RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rc_receiver_if/msg/rosidl_generator_c__visibility_control.h"

#include "rc_receiver_if/msg/detail/receiver_data__struct.h"

/// Initialize msg/ReceiverData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rc_receiver_if__msg__ReceiverData
 * )) before or use
 * rc_receiver_if__msg__ReceiverData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rc_receiver_if
bool
rc_receiver_if__msg__ReceiverData__init(rc_receiver_if__msg__ReceiverData * msg);

/// Finalize msg/ReceiverData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc_receiver_if
void
rc_receiver_if__msg__ReceiverData__fini(rc_receiver_if__msg__ReceiverData * msg);

/// Create msg/ReceiverData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rc_receiver_if__msg__ReceiverData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rc_receiver_if
rc_receiver_if__msg__ReceiverData *
rc_receiver_if__msg__ReceiverData__create();

/// Destroy msg/ReceiverData message.
/**
 * It calls
 * rc_receiver_if__msg__ReceiverData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc_receiver_if
void
rc_receiver_if__msg__ReceiverData__destroy(rc_receiver_if__msg__ReceiverData * msg);

/// Check for msg/ReceiverData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc_receiver_if
bool
rc_receiver_if__msg__ReceiverData__are_equal(const rc_receiver_if__msg__ReceiverData * lhs, const rc_receiver_if__msg__ReceiverData * rhs);

/// Copy a msg/ReceiverData message.
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
ROSIDL_GENERATOR_C_PUBLIC_rc_receiver_if
bool
rc_receiver_if__msg__ReceiverData__copy(
  const rc_receiver_if__msg__ReceiverData * input,
  rc_receiver_if__msg__ReceiverData * output);

/// Initialize array of msg/ReceiverData messages.
/**
 * It allocates the memory for the number of elements and calls
 * rc_receiver_if__msg__ReceiverData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc_receiver_if
bool
rc_receiver_if__msg__ReceiverData__Sequence__init(rc_receiver_if__msg__ReceiverData__Sequence * array, size_t size);

/// Finalize array of msg/ReceiverData messages.
/**
 * It calls
 * rc_receiver_if__msg__ReceiverData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc_receiver_if
void
rc_receiver_if__msg__ReceiverData__Sequence__fini(rc_receiver_if__msg__ReceiverData__Sequence * array);

/// Create array of msg/ReceiverData messages.
/**
 * It allocates the memory for the array and calls
 * rc_receiver_if__msg__ReceiverData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rc_receiver_if
rc_receiver_if__msg__ReceiverData__Sequence *
rc_receiver_if__msg__ReceiverData__Sequence__create(size_t size);

/// Destroy array of msg/ReceiverData messages.
/**
 * It calls
 * rc_receiver_if__msg__ReceiverData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc_receiver_if
void
rc_receiver_if__msg__ReceiverData__Sequence__destroy(rc_receiver_if__msg__ReceiverData__Sequence * array);

/// Check for msg/ReceiverData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc_receiver_if
bool
rc_receiver_if__msg__ReceiverData__Sequence__are_equal(const rc_receiver_if__msg__ReceiverData__Sequence * lhs, const rc_receiver_if__msg__ReceiverData__Sequence * rhs);

/// Copy an array of msg/ReceiverData messages.
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
ROSIDL_GENERATOR_C_PUBLIC_rc_receiver_if
bool
rc_receiver_if__msg__ReceiverData__Sequence__copy(
  const rc_receiver_if__msg__ReceiverData__Sequence * input,
  rc_receiver_if__msg__ReceiverData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__FUNCTIONS_H_
