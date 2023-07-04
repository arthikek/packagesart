// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from raspimouse_msgs:msg/Leds.idl
// generated code does not contain a copyright notice

#ifndef RASPIMOUSE_MSGS__MSG__DETAIL__LEDS__TRAITS_HPP_
#define RASPIMOUSE_MSGS__MSG__DETAIL__LEDS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "raspimouse_msgs/msg/detail/leds__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace raspimouse_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Leds & msg,
  std::ostream & out)
{
  out << "{";
  // member: led0
  {
    out << "led0: ";
    rosidl_generator_traits::value_to_yaml(msg.led0, out);
    out << ", ";
  }

  // member: led1
  {
    out << "led1: ";
    rosidl_generator_traits::value_to_yaml(msg.led1, out);
    out << ", ";
  }

  // member: led2
  {
    out << "led2: ";
    rosidl_generator_traits::value_to_yaml(msg.led2, out);
    out << ", ";
  }

  // member: led3
  {
    out << "led3: ";
    rosidl_generator_traits::value_to_yaml(msg.led3, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Leds & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: led0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "led0: ";
    rosidl_generator_traits::value_to_yaml(msg.led0, out);
    out << "\n";
  }

  // member: led1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "led1: ";
    rosidl_generator_traits::value_to_yaml(msg.led1, out);
    out << "\n";
  }

  // member: led2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "led2: ";
    rosidl_generator_traits::value_to_yaml(msg.led2, out);
    out << "\n";
  }

  // member: led3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "led3: ";
    rosidl_generator_traits::value_to_yaml(msg.led3, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Leds & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace raspimouse_msgs

namespace rosidl_generator_traits
{

[[deprecated("use raspimouse_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const raspimouse_msgs::msg::Leds & msg,
  std::ostream & out, size_t indentation = 0)
{
  raspimouse_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use raspimouse_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const raspimouse_msgs::msg::Leds & msg)
{
  return raspimouse_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<raspimouse_msgs::msg::Leds>()
{
  return "raspimouse_msgs::msg::Leds";
}

template<>
inline const char * name<raspimouse_msgs::msg::Leds>()
{
  return "raspimouse_msgs/msg/Leds";
}

template<>
struct has_fixed_size<raspimouse_msgs::msg::Leds>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<raspimouse_msgs::msg::Leds>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<raspimouse_msgs::msg::Leds>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RASPIMOUSE_MSGS__MSG__DETAIL__LEDS__TRAITS_HPP_
