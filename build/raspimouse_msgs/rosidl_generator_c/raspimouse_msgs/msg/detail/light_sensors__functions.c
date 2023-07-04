// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from raspimouse_msgs:msg/LightSensors.idl
// generated code does not contain a copyright notice
#include "raspimouse_msgs/msg/detail/light_sensors__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
raspimouse_msgs__msg__LightSensors__init(raspimouse_msgs__msg__LightSensors * msg)
{
  if (!msg) {
    return false;
  }
  // forward_r
  // forward_l
  // left
  // right
  return true;
}

void
raspimouse_msgs__msg__LightSensors__fini(raspimouse_msgs__msg__LightSensors * msg)
{
  if (!msg) {
    return;
  }
  // forward_r
  // forward_l
  // left
  // right
}

bool
raspimouse_msgs__msg__LightSensors__are_equal(const raspimouse_msgs__msg__LightSensors * lhs, const raspimouse_msgs__msg__LightSensors * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // forward_r
  if (lhs->forward_r != rhs->forward_r) {
    return false;
  }
  // forward_l
  if (lhs->forward_l != rhs->forward_l) {
    return false;
  }
  // left
  if (lhs->left != rhs->left) {
    return false;
  }
  // right
  if (lhs->right != rhs->right) {
    return false;
  }
  return true;
}

bool
raspimouse_msgs__msg__LightSensors__copy(
  const raspimouse_msgs__msg__LightSensors * input,
  raspimouse_msgs__msg__LightSensors * output)
{
  if (!input || !output) {
    return false;
  }
  // forward_r
  output->forward_r = input->forward_r;
  // forward_l
  output->forward_l = input->forward_l;
  // left
  output->left = input->left;
  // right
  output->right = input->right;
  return true;
}

raspimouse_msgs__msg__LightSensors *
raspimouse_msgs__msg__LightSensors__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  raspimouse_msgs__msg__LightSensors * msg = (raspimouse_msgs__msg__LightSensors *)allocator.allocate(sizeof(raspimouse_msgs__msg__LightSensors), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(raspimouse_msgs__msg__LightSensors));
  bool success = raspimouse_msgs__msg__LightSensors__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
raspimouse_msgs__msg__LightSensors__destroy(raspimouse_msgs__msg__LightSensors * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    raspimouse_msgs__msg__LightSensors__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
raspimouse_msgs__msg__LightSensors__Sequence__init(raspimouse_msgs__msg__LightSensors__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  raspimouse_msgs__msg__LightSensors * data = NULL;

  if (size) {
    data = (raspimouse_msgs__msg__LightSensors *)allocator.zero_allocate(size, sizeof(raspimouse_msgs__msg__LightSensors), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = raspimouse_msgs__msg__LightSensors__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        raspimouse_msgs__msg__LightSensors__fini(&data[i - 1]);
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
raspimouse_msgs__msg__LightSensors__Sequence__fini(raspimouse_msgs__msg__LightSensors__Sequence * array)
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
      raspimouse_msgs__msg__LightSensors__fini(&array->data[i]);
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

raspimouse_msgs__msg__LightSensors__Sequence *
raspimouse_msgs__msg__LightSensors__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  raspimouse_msgs__msg__LightSensors__Sequence * array = (raspimouse_msgs__msg__LightSensors__Sequence *)allocator.allocate(sizeof(raspimouse_msgs__msg__LightSensors__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = raspimouse_msgs__msg__LightSensors__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
raspimouse_msgs__msg__LightSensors__Sequence__destroy(raspimouse_msgs__msg__LightSensors__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    raspimouse_msgs__msg__LightSensors__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
raspimouse_msgs__msg__LightSensors__Sequence__are_equal(const raspimouse_msgs__msg__LightSensors__Sequence * lhs, const raspimouse_msgs__msg__LightSensors__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!raspimouse_msgs__msg__LightSensors__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
raspimouse_msgs__msg__LightSensors__Sequence__copy(
  const raspimouse_msgs__msg__LightSensors__Sequence * input,
  raspimouse_msgs__msg__LightSensors__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(raspimouse_msgs__msg__LightSensors);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    raspimouse_msgs__msg__LightSensors * data =
      (raspimouse_msgs__msg__LightSensors *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!raspimouse_msgs__msg__LightSensors__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          raspimouse_msgs__msg__LightSensors__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!raspimouse_msgs__msg__LightSensors__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
