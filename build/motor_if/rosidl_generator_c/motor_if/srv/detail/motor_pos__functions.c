// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motor_if:srv/MotorPos.idl
// generated code does not contain a copyright notice
#include "motor_if/srv/detail/motor_pos__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
motor_if__srv__MotorPos_Request__init(motor_if__srv__MotorPos_Request * msg)
{
  if (!msg) {
    return false;
  }
  // linear
  // angular
  return true;
}

void
motor_if__srv__MotorPos_Request__fini(motor_if__srv__MotorPos_Request * msg)
{
  if (!msg) {
    return;
  }
  // linear
  // angular
}

bool
motor_if__srv__MotorPos_Request__are_equal(const motor_if__srv__MotorPos_Request * lhs, const motor_if__srv__MotorPos_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // linear
  if (lhs->linear != rhs->linear) {
    return false;
  }
  // angular
  if (lhs->angular != rhs->angular) {
    return false;
  }
  return true;
}

bool
motor_if__srv__MotorPos_Request__copy(
  const motor_if__srv__MotorPos_Request * input,
  motor_if__srv__MotorPos_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // linear
  output->linear = input->linear;
  // angular
  output->angular = input->angular;
  return true;
}

motor_if__srv__MotorPos_Request *
motor_if__srv__MotorPos_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_if__srv__MotorPos_Request * msg = (motor_if__srv__MotorPos_Request *)allocator.allocate(sizeof(motor_if__srv__MotorPos_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_if__srv__MotorPos_Request));
  bool success = motor_if__srv__MotorPos_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_if__srv__MotorPos_Request__destroy(motor_if__srv__MotorPos_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_if__srv__MotorPos_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_if__srv__MotorPos_Request__Sequence__init(motor_if__srv__MotorPos_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_if__srv__MotorPos_Request * data = NULL;

  if (size) {
    data = (motor_if__srv__MotorPos_Request *)allocator.zero_allocate(size, sizeof(motor_if__srv__MotorPos_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_if__srv__MotorPos_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_if__srv__MotorPos_Request__fini(&data[i - 1]);
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
motor_if__srv__MotorPos_Request__Sequence__fini(motor_if__srv__MotorPos_Request__Sequence * array)
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
      motor_if__srv__MotorPos_Request__fini(&array->data[i]);
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

motor_if__srv__MotorPos_Request__Sequence *
motor_if__srv__MotorPos_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_if__srv__MotorPos_Request__Sequence * array = (motor_if__srv__MotorPos_Request__Sequence *)allocator.allocate(sizeof(motor_if__srv__MotorPos_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_if__srv__MotorPos_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_if__srv__MotorPos_Request__Sequence__destroy(motor_if__srv__MotorPos_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_if__srv__MotorPos_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_if__srv__MotorPos_Request__Sequence__are_equal(const motor_if__srv__MotorPos_Request__Sequence * lhs, const motor_if__srv__MotorPos_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_if__srv__MotorPos_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_if__srv__MotorPos_Request__Sequence__copy(
  const motor_if__srv__MotorPos_Request__Sequence * input,
  motor_if__srv__MotorPos_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_if__srv__MotorPos_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_if__srv__MotorPos_Request * data =
      (motor_if__srv__MotorPos_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_if__srv__MotorPos_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_if__srv__MotorPos_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_if__srv__MotorPos_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
motor_if__srv__MotorPos_Response__init(motor_if__srv__MotorPos_Response * msg)
{
  if (!msg) {
    return false;
  }
  // result
  return true;
}

void
motor_if__srv__MotorPos_Response__fini(motor_if__srv__MotorPos_Response * msg)
{
  if (!msg) {
    return;
  }
  // result
}

bool
motor_if__srv__MotorPos_Response__are_equal(const motor_if__srv__MotorPos_Response * lhs, const motor_if__srv__MotorPos_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // result
  if (lhs->result != rhs->result) {
    return false;
  }
  return true;
}

bool
motor_if__srv__MotorPos_Response__copy(
  const motor_if__srv__MotorPos_Response * input,
  motor_if__srv__MotorPos_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // result
  output->result = input->result;
  return true;
}

motor_if__srv__MotorPos_Response *
motor_if__srv__MotorPos_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_if__srv__MotorPos_Response * msg = (motor_if__srv__MotorPos_Response *)allocator.allocate(sizeof(motor_if__srv__MotorPos_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_if__srv__MotorPos_Response));
  bool success = motor_if__srv__MotorPos_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_if__srv__MotorPos_Response__destroy(motor_if__srv__MotorPos_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_if__srv__MotorPos_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_if__srv__MotorPos_Response__Sequence__init(motor_if__srv__MotorPos_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_if__srv__MotorPos_Response * data = NULL;

  if (size) {
    data = (motor_if__srv__MotorPos_Response *)allocator.zero_allocate(size, sizeof(motor_if__srv__MotorPos_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_if__srv__MotorPos_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_if__srv__MotorPos_Response__fini(&data[i - 1]);
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
motor_if__srv__MotorPos_Response__Sequence__fini(motor_if__srv__MotorPos_Response__Sequence * array)
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
      motor_if__srv__MotorPos_Response__fini(&array->data[i]);
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

motor_if__srv__MotorPos_Response__Sequence *
motor_if__srv__MotorPos_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_if__srv__MotorPos_Response__Sequence * array = (motor_if__srv__MotorPos_Response__Sequence *)allocator.allocate(sizeof(motor_if__srv__MotorPos_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_if__srv__MotorPos_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_if__srv__MotorPos_Response__Sequence__destroy(motor_if__srv__MotorPos_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_if__srv__MotorPos_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_if__srv__MotorPos_Response__Sequence__are_equal(const motor_if__srv__MotorPos_Response__Sequence * lhs, const motor_if__srv__MotorPos_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_if__srv__MotorPos_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_if__srv__MotorPos_Response__Sequence__copy(
  const motor_if__srv__MotorPos_Response__Sequence * input,
  motor_if__srv__MotorPos_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_if__srv__MotorPos_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_if__srv__MotorPos_Response * data =
      (motor_if__srv__MotorPos_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_if__srv__MotorPos_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_if__srv__MotorPos_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_if__srv__MotorPos_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
