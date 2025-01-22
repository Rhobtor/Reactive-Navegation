// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from car_cpp:srv/SetBias.idl
// generated code does not contain a copyright notice
#include "car_cpp/srv/detail/set_bias__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `bias`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
car_cpp__srv__SetBias_Request__init(car_cpp__srv__SetBias_Request * msg)
{
  if (!msg) {
    return false;
  }
  // bias
  if (!geometry_msgs__msg__Vector3__init(&msg->bias)) {
    car_cpp__srv__SetBias_Request__fini(msg);
    return false;
  }
  return true;
}

void
car_cpp__srv__SetBias_Request__fini(car_cpp__srv__SetBias_Request * msg)
{
  if (!msg) {
    return;
  }
  // bias
  geometry_msgs__msg__Vector3__fini(&msg->bias);
}

bool
car_cpp__srv__SetBias_Request__are_equal(const car_cpp__srv__SetBias_Request * lhs, const car_cpp__srv__SetBias_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bias
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->bias), &(rhs->bias)))
  {
    return false;
  }
  return true;
}

bool
car_cpp__srv__SetBias_Request__copy(
  const car_cpp__srv__SetBias_Request * input,
  car_cpp__srv__SetBias_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // bias
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->bias), &(output->bias)))
  {
    return false;
  }
  return true;
}

car_cpp__srv__SetBias_Request *
car_cpp__srv__SetBias_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  car_cpp__srv__SetBias_Request * msg = (car_cpp__srv__SetBias_Request *)allocator.allocate(sizeof(car_cpp__srv__SetBias_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(car_cpp__srv__SetBias_Request));
  bool success = car_cpp__srv__SetBias_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
car_cpp__srv__SetBias_Request__destroy(car_cpp__srv__SetBias_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    car_cpp__srv__SetBias_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
car_cpp__srv__SetBias_Request__Sequence__init(car_cpp__srv__SetBias_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  car_cpp__srv__SetBias_Request * data = NULL;

  if (size) {
    data = (car_cpp__srv__SetBias_Request *)allocator.zero_allocate(size, sizeof(car_cpp__srv__SetBias_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = car_cpp__srv__SetBias_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        car_cpp__srv__SetBias_Request__fini(&data[i - 1]);
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
car_cpp__srv__SetBias_Request__Sequence__fini(car_cpp__srv__SetBias_Request__Sequence * array)
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
      car_cpp__srv__SetBias_Request__fini(&array->data[i]);
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

car_cpp__srv__SetBias_Request__Sequence *
car_cpp__srv__SetBias_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  car_cpp__srv__SetBias_Request__Sequence * array = (car_cpp__srv__SetBias_Request__Sequence *)allocator.allocate(sizeof(car_cpp__srv__SetBias_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = car_cpp__srv__SetBias_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
car_cpp__srv__SetBias_Request__Sequence__destroy(car_cpp__srv__SetBias_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    car_cpp__srv__SetBias_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
car_cpp__srv__SetBias_Request__Sequence__are_equal(const car_cpp__srv__SetBias_Request__Sequence * lhs, const car_cpp__srv__SetBias_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!car_cpp__srv__SetBias_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
car_cpp__srv__SetBias_Request__Sequence__copy(
  const car_cpp__srv__SetBias_Request__Sequence * input,
  car_cpp__srv__SetBias_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(car_cpp__srv__SetBias_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    car_cpp__srv__SetBias_Request * data =
      (car_cpp__srv__SetBias_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!car_cpp__srv__SetBias_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          car_cpp__srv__SetBias_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!car_cpp__srv__SetBias_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
car_cpp__srv__SetBias_Response__init(car_cpp__srv__SetBias_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
car_cpp__srv__SetBias_Response__fini(car_cpp__srv__SetBias_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
car_cpp__srv__SetBias_Response__are_equal(const car_cpp__srv__SetBias_Response * lhs, const car_cpp__srv__SetBias_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
car_cpp__srv__SetBias_Response__copy(
  const car_cpp__srv__SetBias_Response * input,
  car_cpp__srv__SetBias_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

car_cpp__srv__SetBias_Response *
car_cpp__srv__SetBias_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  car_cpp__srv__SetBias_Response * msg = (car_cpp__srv__SetBias_Response *)allocator.allocate(sizeof(car_cpp__srv__SetBias_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(car_cpp__srv__SetBias_Response));
  bool success = car_cpp__srv__SetBias_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
car_cpp__srv__SetBias_Response__destroy(car_cpp__srv__SetBias_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    car_cpp__srv__SetBias_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
car_cpp__srv__SetBias_Response__Sequence__init(car_cpp__srv__SetBias_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  car_cpp__srv__SetBias_Response * data = NULL;

  if (size) {
    data = (car_cpp__srv__SetBias_Response *)allocator.zero_allocate(size, sizeof(car_cpp__srv__SetBias_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = car_cpp__srv__SetBias_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        car_cpp__srv__SetBias_Response__fini(&data[i - 1]);
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
car_cpp__srv__SetBias_Response__Sequence__fini(car_cpp__srv__SetBias_Response__Sequence * array)
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
      car_cpp__srv__SetBias_Response__fini(&array->data[i]);
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

car_cpp__srv__SetBias_Response__Sequence *
car_cpp__srv__SetBias_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  car_cpp__srv__SetBias_Response__Sequence * array = (car_cpp__srv__SetBias_Response__Sequence *)allocator.allocate(sizeof(car_cpp__srv__SetBias_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = car_cpp__srv__SetBias_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
car_cpp__srv__SetBias_Response__Sequence__destroy(car_cpp__srv__SetBias_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    car_cpp__srv__SetBias_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
car_cpp__srv__SetBias_Response__Sequence__are_equal(const car_cpp__srv__SetBias_Response__Sequence * lhs, const car_cpp__srv__SetBias_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!car_cpp__srv__SetBias_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
car_cpp__srv__SetBias_Response__Sequence__copy(
  const car_cpp__srv__SetBias_Response__Sequence * input,
  car_cpp__srv__SetBias_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(car_cpp__srv__SetBias_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    car_cpp__srv__SetBias_Response * data =
      (car_cpp__srv__SetBias_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!car_cpp__srv__SetBias_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          car_cpp__srv__SetBias_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!car_cpp__srv__SetBias_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
