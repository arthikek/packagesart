// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from raspimouse_msgs:msg/Leds.idl
// generated code does not contain a copyright notice

#ifndef RASPIMOUSE_MSGS__MSG__DETAIL__LEDS__STRUCT_H_
#define RASPIMOUSE_MSGS__MSG__DETAIL__LEDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Leds in the package raspimouse_msgs.
typedef struct raspimouse_msgs__msg__Leds
{
  bool led0;
  bool led1;
  bool led2;
  bool led3;
} raspimouse_msgs__msg__Leds;

// Struct for a sequence of raspimouse_msgs__msg__Leds.
typedef struct raspimouse_msgs__msg__Leds__Sequence
{
  raspimouse_msgs__msg__Leds * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} raspimouse_msgs__msg__Leds__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RASPIMOUSE_MSGS__MSG__DETAIL__LEDS__STRUCT_H_
