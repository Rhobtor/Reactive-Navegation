// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from car_interfaces:msg/Graph.idl
// generated code does not contain a copyright notice
#include "car_interfaces/msg/detail/graph__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `nodes`
#include "geometry_msgs/msg/detail/pose_array__functions.h"
// Member `edges`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
car_interfaces__msg__Graph__init(car_interfaces__msg__Graph * msg)
{
  if (!msg) {
    return false;
  }
  // nodes
  if (!geometry_msgs__msg__PoseArray__init(&msg->nodes)) {
    car_interfaces__msg__Graph__fini(msg);
    return false;
  }
  // edges
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->edges, 0)) {
    car_interfaces__msg__Graph__fini(msg);
    return false;
  }
  return true;
}

void
car_interfaces__msg__Graph__fini(car_interfaces__msg__Graph * msg)
{
  if (!msg) {
    return;
  }
  // nodes
  geometry_msgs__msg__PoseArray__fini(&msg->nodes);
  // edges
  rosidl_runtime_c__int32__Sequence__fini(&msg->edges);
}

bool
car_interfaces__msg__Graph__are_equal(const car_interfaces__msg__Graph * lhs, const car_interfaces__msg__Graph * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // nodes
  if (!geometry_msgs__msg__PoseArray__are_equal(
      &(lhs->nodes), &(rhs->nodes)))
  {
    return false;
  }
  // edges
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->edges), &(rhs->edges)))
  {
    return false;
  }
  return true;
}

bool
car_interfaces__msg__Graph__copy(
  const car_interfaces__msg__Graph * input,
  car_interfaces__msg__Graph * output)
{
  if (!input || !output) {
    return false;
  }
  // nodes
  if (!geometry_msgs__msg__PoseArray__copy(
      &(input->nodes), &(output->nodes)))
  {
    return false;
  }
  // edges
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->edges), &(output->edges)))
  {
    return false;
  }
  return true;
}

car_interfaces__msg__Graph *
car_interfaces__msg__Graph__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  car_interfaces__msg__Graph * msg = (car_interfaces__msg__Graph *)allocator.allocate(sizeof(car_interfaces__msg__Graph), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(car_interfaces__msg__Graph));
  bool success = car_interfaces__msg__Graph__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
car_interfaces__msg__Graph__destroy(car_interfaces__msg__Graph * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    car_interfaces__msg__Graph__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
car_interfaces__msg__Graph__Sequence__init(car_interfaces__msg__Graph__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  car_interfaces__msg__Graph * data = NULL;

  if (size) {
    data = (car_interfaces__msg__Graph *)allocator.zero_allocate(size, sizeof(car_interfaces__msg__Graph), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = car_interfaces__msg__Graph__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        car_interfaces__msg__Graph__fini(&data[i - 1]);
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
car_interfaces__msg__Graph__Sequence__fini(car_interfaces__msg__Graph__Sequence * array)
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
      car_interfaces__msg__Graph__fini(&array->data[i]);
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

car_interfaces__msg__Graph__Sequence *
car_interfaces__msg__Graph__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  car_interfaces__msg__Graph__Sequence * array = (car_interfaces__msg__Graph__Sequence *)allocator.allocate(sizeof(car_interfaces__msg__Graph__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = car_interfaces__msg__Graph__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
car_interfaces__msg__Graph__Sequence__destroy(car_interfaces__msg__Graph__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    car_interfaces__msg__Graph__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
car_interfaces__msg__Graph__Sequence__are_equal(const car_interfaces__msg__Graph__Sequence * lhs, const car_interfaces__msg__Graph__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!car_interfaces__msg__Graph__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
car_interfaces__msg__Graph__Sequence__copy(
  const car_interfaces__msg__Graph__Sequence * input,
  car_interfaces__msg__Graph__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(car_interfaces__msg__Graph);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    car_interfaces__msg__Graph * data =
      (car_interfaces__msg__Graph *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!car_interfaces__msg__Graph__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          car_interfaces__msg__Graph__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!car_interfaces__msg__Graph__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
