// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rc_receiver_if:msg/ReceiverData.idl
// generated code does not contain a copyright notice
#include "rc_receiver_if/msg/detail/receiver_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rc_receiver_if__msg__ReceiverData__init(rc_receiver_if__msg__ReceiverData * msg)
{
  if (!msg) {
    return false;
  }
  // channel_data
  msg->channel_data[0] = 1000;
  msg->channel_data[1] = 1000;
  msg->channel_data[2] = 1000;
  msg->channel_data[3] = 1000;
  msg->channel_data[4] = 1000;
  msg->channel_data[5] = 1000;
  msg->channel_data[6] = 1000;
  msg->channel_data[7] = 1000;
  msg->channel_data[8] = 1000;
  msg->channel_data[9] = 1000;
  msg->channel_data[10] = 1000;
  msg->channel_data[11] = 1000;
  msg->channel_data[12] = 1000;
  msg->channel_data[13] = 1000;
  msg->channel_data[14] = 1000;
  msg->channel_data[15] = 1000;
  msg->channel_data[16] = 1000;
  msg->channel_data[17] = 1000;
  // wheel_control
  msg->wheel_control[0] = 0.0f;
  msg->wheel_control[1] = 0.0f;
  // twist
  msg->twist[0] = 0.0f;
  msg->twist[1] = 0.0f;
  return true;
}

void
rc_receiver_if__msg__ReceiverData__fini(rc_receiver_if__msg__ReceiverData * msg)
{
  if (!msg) {
    return;
  }
  // channel_data
  // wheel_control
  // twist
}

bool
rc_receiver_if__msg__ReceiverData__are_equal(const rc_receiver_if__msg__ReceiverData * lhs, const rc_receiver_if__msg__ReceiverData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // channel_data
  for (size_t i = 0; i < 18; ++i) {
    if (lhs->channel_data[i] != rhs->channel_data[i]) {
      return false;
    }
  }
  // wheel_control
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->wheel_control[i] != rhs->wheel_control[i]) {
      return false;
    }
  }
  // twist
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->twist[i] != rhs->twist[i]) {
      return false;
    }
  }
  return true;
}

bool
rc_receiver_if__msg__ReceiverData__copy(
  const rc_receiver_if__msg__ReceiverData * input,
  rc_receiver_if__msg__ReceiverData * output)
{
  if (!input || !output) {
    return false;
  }
  // channel_data
  for (size_t i = 0; i < 18; ++i) {
    output->channel_data[i] = input->channel_data[i];
  }
  // wheel_control
  for (size_t i = 0; i < 2; ++i) {
    output->wheel_control[i] = input->wheel_control[i];
  }
  // twist
  for (size_t i = 0; i < 2; ++i) {
    output->twist[i] = input->twist[i];
  }
  return true;
}

rc_receiver_if__msg__ReceiverData *
rc_receiver_if__msg__ReceiverData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc_receiver_if__msg__ReceiverData * msg = (rc_receiver_if__msg__ReceiverData *)allocator.allocate(sizeof(rc_receiver_if__msg__ReceiverData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rc_receiver_if__msg__ReceiverData));
  bool success = rc_receiver_if__msg__ReceiverData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rc_receiver_if__msg__ReceiverData__destroy(rc_receiver_if__msg__ReceiverData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rc_receiver_if__msg__ReceiverData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rc_receiver_if__msg__ReceiverData__Sequence__init(rc_receiver_if__msg__ReceiverData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc_receiver_if__msg__ReceiverData * data = NULL;

  if (size) {
    data = (rc_receiver_if__msg__ReceiverData *)allocator.zero_allocate(size, sizeof(rc_receiver_if__msg__ReceiverData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rc_receiver_if__msg__ReceiverData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rc_receiver_if__msg__ReceiverData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rc_receiver_if__msg__ReceiverData__Sequence__fini(rc_receiver_if__msg__ReceiverData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rc_receiver_if__msg__ReceiverData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rc_receiver_if__msg__ReceiverData__Sequence *
rc_receiver_if__msg__ReceiverData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc_receiver_if__msg__ReceiverData__Sequence * array = (rc_receiver_if__msg__ReceiverData__Sequence *)allocator.allocate(sizeof(rc_receiver_if__msg__ReceiverData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rc_receiver_if__msg__ReceiverData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rc_receiver_if__msg__ReceiverData__Sequence__destroy(rc_receiver_if__msg__ReceiverData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rc_receiver_if__msg__ReceiverData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rc_receiver_if__msg__ReceiverData__Sequence__are_equal(const rc_receiver_if__msg__ReceiverData__Sequence * lhs, const rc_receiver_if__msg__ReceiverData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rc_receiver_if__msg__ReceiverData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rc_receiver_if__msg__ReceiverData__Sequence__copy(
  const rc_receiver_if__msg__ReceiverData__Sequence * input,
  rc_receiver_if__msg__ReceiverData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rc_receiver_if__msg__ReceiverData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rc_receiver_if__msg__ReceiverData * data =
      (rc_receiver_if__msg__ReceiverData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rc_receiver_if__msg__ReceiverData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rc_receiver_if__msg__ReceiverData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rc_receiver_if__msg__ReceiverData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
