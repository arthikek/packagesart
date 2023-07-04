// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from raspimouse_msgs:msg/Leds.idl
// generated code does not contain a copyright notice
#include "raspimouse_msgs/msg/detail/leds__rosidl_typesupport_fastrtps_cpp.hpp"
#include "raspimouse_msgs/msg/detail/leds__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace raspimouse_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_raspimouse_msgs
cdr_serialize(
  const raspimouse_msgs::msg::Leds & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: led0
  cdr << (ros_message.led0 ? true : false);
  // Member: led1
  cdr << (ros_message.led1 ? true : false);
  // Member: led2
  cdr << (ros_message.led2 ? true : false);
  // Member: led3
  cdr << (ros_message.led3 ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_raspimouse_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  raspimouse_msgs::msg::Leds & ros_message)
{
  // Member: led0
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.led0 = tmp ? true : false;
  }

  // Member: led1
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.led1 = tmp ? true : false;
  }

  // Member: led2
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.led2 = tmp ? true : false;
  }

  // Member: led3
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.led3 = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_raspimouse_msgs
get_serialized_size(
  const raspimouse_msgs::msg::Leds & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: led0
  {
    size_t item_size = sizeof(ros_message.led0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: led1
  {
    size_t item_size = sizeof(ros_message.led1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: led2
  {
    size_t item_size = sizeof(ros_message.led2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: led3
  {
    size_t item_size = sizeof(ros_message.led3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_raspimouse_msgs
max_serialized_size_Leds(
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


  // Member: led0
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: led1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: led2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: led3
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _Leds__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const raspimouse_msgs::msg::Leds *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Leds__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<raspimouse_msgs::msg::Leds *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Leds__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const raspimouse_msgs::msg::Leds *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Leds__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Leds(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Leds__callbacks = {
  "raspimouse_msgs::msg",
  "Leds",
  _Leds__cdr_serialize,
  _Leds__cdr_deserialize,
  _Leds__get_serialized_size,
  _Leds__max_serialized_size
};

static rosidl_message_type_support_t _Leds__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Leds__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace raspimouse_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_raspimouse_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<raspimouse_msgs::msg::Leds>()
{
  return &raspimouse_msgs::msg::typesupport_fastrtps_cpp::_Leds__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, raspimouse_msgs, msg, Leds)() {
  return &raspimouse_msgs::msg::typesupport_fastrtps_cpp::_Leds__handle;
}

#ifdef __cplusplus
}
#endif
