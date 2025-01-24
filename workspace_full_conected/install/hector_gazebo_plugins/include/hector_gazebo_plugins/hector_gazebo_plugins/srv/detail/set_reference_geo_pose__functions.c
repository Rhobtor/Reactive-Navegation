// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hector_gazebo_plugins:srv/SetReferenceGeoPose.idl
// generated code does not contain a copyright notice
#include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `geo_pose`
#include "geographic_msgs/msg/detail/geo_pose__functions.h"

bool
hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__init(hector_gazebo_plugins__srv__SetReferenceGeoPose_Request * msg)
{
  if (!msg) {
    return false;
  }
  // geo_pose
  if (!geographic_msgs__msg__GeoPose__init(&msg->geo_pose)) {
    hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__fini(msg);
    return false;
  }
  return true;
}

void
hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__fini(hector_gazebo_plugins__srv__SetReferenceGeoPose_Request * msg)
{
  if (!msg) {
    return;
  }
  // geo_pose
  geographic_msgs__msg__GeoPose__fini(&msg->geo_pose);
}

bool
hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__are_equal(const hector_gazebo_plugins__srv__SetReferenceGeoPose_Request * lhs, const hector_gazebo_plugins__srv__SetReferenceGeoPose_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // geo_pose
  if (!geographic_msgs__msg__GeoPose__are_equal(
      &(lhs->geo_pose), &(rhs->geo_pose)))
  {
    return false;
  }
  return true;
}

bool
hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__copy(
  const hector_gazebo_plugins__srv__SetReferenceGeoPose_Request * input,
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // geo_pose
  if (!geographic_msgs__msg__GeoPose__copy(
      &(input->geo_pose), &(output->geo_pose)))
  {
    return false;
  }
  return true;
}

hector_gazebo_plugins__srv__SetReferenceGeoPose_Request *
hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request * msg = (hector_gazebo_plugins__srv__SetReferenceGeoPose_Request *)allocator.allocate(sizeof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Request));
  bool success = hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__destroy(hector_gazebo_plugins__srv__SetReferenceGeoPose_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__init(hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request * data = NULL;

  if (size) {
    data = (hector_gazebo_plugins__srv__SetReferenceGeoPose_Request *)allocator.zero_allocate(size, sizeof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__fini(&data[i - 1]);
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
hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__fini(hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence * array)
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
      hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__fini(&array->data[i]);
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

hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence *
hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence * array = (hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence *)allocator.allocate(sizeof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__destroy(hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__are_equal(const hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence * lhs, const hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__copy(
  const hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence * input,
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hector_gazebo_plugins__srv__SetReferenceGeoPose_Request * data =
      (hector_gazebo_plugins__srv__SetReferenceGeoPose_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__init(hector_gazebo_plugins__srv__SetReferenceGeoPose_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__fini(hector_gazebo_plugins__srv__SetReferenceGeoPose_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__are_equal(const hector_gazebo_plugins__srv__SetReferenceGeoPose_Response * lhs, const hector_gazebo_plugins__srv__SetReferenceGeoPose_Response * rhs)
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
hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__copy(
  const hector_gazebo_plugins__srv__SetReferenceGeoPose_Response * input,
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

hector_gazebo_plugins__srv__SetReferenceGeoPose_Response *
hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Response * msg = (hector_gazebo_plugins__srv__SetReferenceGeoPose_Response *)allocator.allocate(sizeof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Response));
  bool success = hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__destroy(hector_gazebo_plugins__srv__SetReferenceGeoPose_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__init(hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Response * data = NULL;

  if (size) {
    data = (hector_gazebo_plugins__srv__SetReferenceGeoPose_Response *)allocator.zero_allocate(size, sizeof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__fini(&data[i - 1]);
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
hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__fini(hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence * array)
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
      hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__fini(&array->data[i]);
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

hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence *
hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence * array = (hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence *)allocator.allocate(sizeof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__destroy(hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__are_equal(const hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence * lhs, const hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__copy(
  const hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence * input,
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hector_gazebo_plugins__srv__SetReferenceGeoPose_Response * data =
      (hector_gazebo_plugins__srv__SetReferenceGeoPose_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
