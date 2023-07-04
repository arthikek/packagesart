// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from raspimouse_msgs:msg/Switches.idl
// generated code does not contain a copyright notice
#include "raspimouse_msgs/msg/detail/switches__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
raspimouse_msgs__msg__Switches__init(raspimouse_msgs__msg__Switches * msg)
{
  if (!msg) {
    return false;
  }
  // switch0
  // switch1
  // switch2
  return true;
}

void
raspimouse_msgs__msg__Switches__fini(raspimouse_msgs__msg__Switches * msg)
{
  if (!msg) {
    return;
  }
  // switch0
  // switch1
  // switch2
}

bool
raspimouse_msgs__msg__Switches__are_equal(const raspimouse_msgs__msg__Switches * lhs, const raspimouse_msgs__msg__Switches * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // switch0
  if (lhs->switch0 != rhs->switch0) {
    return false;
  }
  // switch1
  if (lhs->switch1 != rhs->switch1) {
    return false;
  }
  // switch2
  if (lhs->switch2 != rhs->switch2) {
    return false;
  }
  return true;
}

bool
raspimouse_msgs__msg__Switches__copy(
  const raspimouse_msgs__msg__Switches * input,
  raspimouse_msgs__msg__Switches * output)
{
  if (!input || !output) {
    return false;
  }
  // switch0
  output->switch0 = input->switch0;
  // switch1
  output->switch1 = input->switch1;
  // switch2
  output->switch2 = input->switch2;
  return true;
}

raspimouse_msgs__msg__Switches *
raspimouse_msgs__msg__Switches__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  raspimouse_msgs__msg__Switches * msg = (raspimouse_msgs__msg__Switches *)allocator.allocate(sizeof(raspimouse_msgs__msg__Switches), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(raspimouse_msgs__msg__Switches));
  bool success = raspimouse_msgs__msg__Switches__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
raspimouse_msgs__msg__Switches__destroy(raspimouse_msgs__msg__Switches * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    raspimouse_msgs__msg__Switches__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
raspimouse_msgs__msg__Switches__Sequence__init(raspimouse_msgs__msg__Switches__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  raspimouse_msgs__msg__Switches * data = NULL;

  if (size) {
    data = (raspimouse_msgs__msg__Switches *)allocator.zero_allocate(size, sizeof(raspimouse_msgs__msg__Switches), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = raspimouse_msgs__msg__Switches__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        raspimouse_msgs__msg__Switches__fini(&data[i - 1]);
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
raspimouse_msgs__msg__Switches__Sequence__fini(raspimouse_msgs__msg__Switches__Sequence * array)
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
      raspimouse_msgs__msg__Switches__fini(&array->data[i]);
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

raspimouse_msgs__msg__Switches__Sequence *
raspimouse_msgs__msg__Switches__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  raspimouse_msgs__msg__Switches__Sequence * array = (raspimouse_msgs__msg__Switches__Sequence *)allocator.allocate(sizeof(raspimouse_msgs__msg__Switches__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = raspimouse_msgs__msg__Switches__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
raspimouse_msgs__msg__Switches__Sequence__destroy(raspimouse_msgs__msg__Switches__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    raspimouse_msgs__msg__Switches__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
raspimouse_msgs__msg__Switches__Sequence__are_equal(const raspimouse_msgs__msg__Switches__Sequence * lhs, const raspimouse_msgs__msg__Switches__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!raspimouse_msgs__msg__Switches__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
raspimouse_msgs__msg__Switches__Sequence__copy(
  const raspimouse_msgs__msg__Switches__Sequence * input,
  raspimouse_msgs__msg__Switches__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(raspimouse_msgs__msg__Switches);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    raspimouse_msgs__msg__Switches * data =
      (raspimouse_msgs__msg__Switches *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!raspimouse_msgs__msg__Switches__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          raspimouse_msgs__msg__Switches__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!raspimouse_msgs__msg__Switches__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
