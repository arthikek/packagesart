// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from raspimouse_msgs:msg/Switches.idl
// generated code does not contain a copyright notice

#ifndef RASPIMOUSE_MSGS__MSG__DETAIL__SWITCHES__STRUCT_H_
#define RASPIMOUSE_MSGS__MSG__DETAIL__SWITCHES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Switches in the package raspimouse_msgs.
typedef struct raspimouse_msgs__msg__Switches
{
  bool switch0;
  bool switch1;
  bool switch2;
} raspimouse_msgs__msg__Switches;

// Struct for a sequence of raspimouse_msgs__msg__Switches.
typedef struct raspimouse_msgs__msg__Switches__Sequence
{
  raspimouse_msgs__msg__Switches * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} raspimouse_msgs__msg__Switches__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RASPIMOUSE_MSGS__MSG__DETAIL__SWITCHES__STRUCT_H_
