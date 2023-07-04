// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from raspimouse_msgs:msg/Switches.idl
// generated code does not contain a copyright notice
#include "raspimouse_msgs/msg/detail/switches__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "raspimouse_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "raspimouse_msgs/msg/detail/switches__struct.h"
#include "raspimouse_msgs/msg/detail/switches__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _Switches__ros_msg_type = raspimouse_msgs__msg__Switches;

static bool _Switches__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Switches__ros_msg_type * ros_message = static_cast<const _Switches__ros_msg_type *>(untyped_ros_message);
  // Field name: switch0
  {
    cdr << (ros_message->switch0 ? true : false);
  }

  // Field name: switch1
  {
    cdr << (ros_message->switch1 ? true : false);
  }

  // Field name: switch2
  {
    cdr << (ros_message->switch2 ? true : false);
  }

  return true;
}

static bool _Switches__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Switches__ros_msg_type * ros_message = static_cast<_Switches__ros_msg_type *>(untyped_ros_message);
  // Field name: switch0
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->switch0 = tmp ? true : false;
  }

  // Field name: switch1
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->switch1 = tmp ? true : false;
  }

  // Field name: switch2
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->switch2 = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_raspimouse_msgs
size_t get_serialized_size_raspimouse_msgs__msg__Switches(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Switches__ros_msg_type * ros_message = static_cast<const _Switches__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name switch0
  {
    size_t item_size = sizeof(ros_message->switch0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name switch1
  {
    size_t item_size = sizeof(ros_message->switch1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name switch2
  {
    size_t item_size = sizeof(ros_message->switch2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Switches__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_raspimouse_msgs__msg__Switches(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_raspimouse_msgs
size_t max_serialized_size_raspimouse_msgs__msg__Switches(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: switch0
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: switch1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: switch2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _Switches__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_raspimouse_msgs__msg__Switches(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Switches = {
  "raspimouse_msgs::msg",
  "Switches",
  _Switches__cdr_serialize,
  _Switches__cdr_deserialize,
  _Switches__get_serialized_size,
  _Switches__max_serialized_size
};

static rosidl_message_type_support_t _Switches__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Switches,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, raspimouse_msgs, msg, Switches)() {
  return &_Switches__type_support;
}

#if defined(__cplusplus)
}
#endif
