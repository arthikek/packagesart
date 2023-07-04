// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from raspimouse_msgs:msg/Leds.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "raspimouse_msgs/msg/detail/leds__rosidl_typesupport_introspection_c.h"
#include "raspimouse_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "raspimouse_msgs/msg/detail/leds__functions.h"
#include "raspimouse_msgs/msg/detail/leds__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void raspimouse_msgs__msg__Leds__rosidl_typesupport_introspection_c__Leds_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  raspimouse_msgs__msg__Leds__init(message_memory);
}

void raspimouse_msgs__msg__Leds__rosidl_typesupport_introspection_c__Leds_fini_function(void * message_memory)
{
  raspimouse_msgs__msg__Leds__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember raspimouse_msgs__msg__Leds__rosidl_typesupport_introspection_c__Leds_message_member_array[4] = {
  {
    "led0",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(raspimouse_msgs__msg__Leds, led0),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "led1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(raspimouse_msgs__msg__Leds, led1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "led2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(raspimouse_msgs__msg__Leds, led2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "led3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(raspimouse_msgs__msg__Leds, led3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers raspimouse_msgs__msg__Leds__rosidl_typesupport_introspection_c__Leds_message_members = {
  "raspimouse_msgs__msg",  // message namespace
  "Leds",  // message name
  4,  // number of fields
  sizeof(raspimouse_msgs__msg__Leds),
  raspimouse_msgs__msg__Leds__rosidl_typesupport_introspection_c__Leds_message_member_array,  // message members
  raspimouse_msgs__msg__Leds__rosidl_typesupport_introspection_c__Leds_init_function,  // function to initialize message memory (memory has to be allocated)
  raspimouse_msgs__msg__Leds__rosidl_typesupport_introspection_c__Leds_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t raspimouse_msgs__msg__Leds__rosidl_typesupport_introspection_c__Leds_message_type_support_handle = {
  0,
  &raspimouse_msgs__msg__Leds__rosidl_typesupport_introspection_c__Leds_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_raspimouse_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, raspimouse_msgs, msg, Leds)() {
  if (!raspimouse_msgs__msg__Leds__rosidl_typesupport_introspection_c__Leds_message_type_support_handle.typesupport_identifier) {
    raspimouse_msgs__msg__Leds__rosidl_typesupport_introspection_c__Leds_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &raspimouse_msgs__msg__Leds__rosidl_typesupport_introspection_c__Leds_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
