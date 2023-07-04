// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from raspimouse_msgs:msg/Leds.idl
// generated code does not contain a copyright notice
#include "raspimouse_msgs/msg/detail/leds__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
raspimouse_msgs__msg__Leds__init(raspimouse_msgs__msg__Leds * msg)
{
  if (!msg) {
    return false;
  }
  // led0
  // led1
  // led2
  // led3
  return true;
}

void
raspimouse_msgs__msg__Leds__fini(raspimouse_msgs__msg__Leds * msg)
{
  if (!msg) {
    return;
  }
  // led0
  // led1
  // led2
  // led3
}

bool
raspimouse_msgs__msg__Leds__are_equal(const raspimouse_msgs__msg__Leds * lhs, const raspimouse_msgs__msg__Leds * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // led0
  if (lhs->led0 != rhs->led0) {
    return false;
  }
  // led1
  if (lhs->led1 != rhs->led1) {
    return false;
  }
  // led2
  if (lhs->led2 != rhs->led2) {
    return false;
  }
  // led3
  if (lhs->led3 != rhs->led3) {
    return false;
  }
  return true;
}

bool
raspimouse_msgs__msg__Leds__copy(
  const raspimouse_msgs__msg__Leds * input,
  raspimouse_msgs__msg__Leds * output)
{
  if (!input || !output) {
    return false;
  }
  // led0
  output->led0 = input->led0;
  // led1
  output->led1 = input->led1;
  // led2
  output->led2 = input->led2;
  // led3
  output->led3 = input->led3;
  return true;
}

raspimouse_msgs__msg__Leds *
raspimouse_msgs__msg__Leds__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  raspimouse_msgs__msg__Leds * msg = (raspimouse_msgs__msg__Leds *)allocator.allocate(sizeof(raspimouse_msgs__msg__Leds), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(raspimouse_msgs__msg__Leds));
  bool success = raspimouse_msgs__msg__Leds__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
raspimouse_msgs__msg__Leds__destroy(raspimouse_msgs__msg__Leds * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    raspimouse_msgs__msg__Leds__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
raspimouse_msgs__msg__Leds__Sequence__init(raspimouse_msgs__msg__Leds__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  raspimouse_msgs__msg__Leds * data = NULL;

  if (size) {
    data = (raspimouse_msgs__msg__Leds *)allocator.zero_allocate(size, sizeof(raspimouse_msgs__msg__Leds), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = raspimouse_msgs__msg__Leds__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        raspimouse_msgs__msg__Leds__fini(&data[i - 1]);
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
raspimouse_msgs__msg__Leds__Sequence__fini(raspimouse_msgs__msg__Leds__Sequence * array)
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
      raspimouse_msgs__msg__Leds__fini(&array->data[i]);
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

raspimouse_msgs__msg__Leds__Sequence *
raspimouse_msgs__msg__Leds__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  raspimouse_msgs__msg__Leds__Sequence * array = (raspimouse_msgs__msg__Leds__Sequence *)allocator.allocate(sizeof(raspimouse_msgs__msg__Leds__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = raspimouse_msgs__msg__Leds__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
raspimouse_msgs__msg__Leds__Sequence__destroy(raspimouse_msgs__msg__Leds__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    raspimouse_msgs__msg__Leds__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
raspimouse_msgs__msg__Leds__Sequence__are_equal(const raspimouse_msgs__msg__Leds__Sequence * lhs, const raspimouse_msgs__msg__Leds__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!raspimouse_msgs__msg__Leds__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
raspimouse_msgs__msg__Leds__Sequence__copy(
  const raspimouse_msgs__msg__Leds__Sequence * input,
  raspimouse_msgs__msg__Leds__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(raspimouse_msgs__msg__Leds);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    raspimouse_msgs__msg__Leds * data =
      (raspimouse_msgs__msg__Leds *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!raspimouse_msgs__msg__Leds__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          raspimouse_msgs__msg__Leds__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!raspimouse_msgs__msg__Leds__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
