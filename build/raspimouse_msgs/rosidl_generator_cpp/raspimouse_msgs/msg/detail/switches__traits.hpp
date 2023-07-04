// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from raspimouse_msgs:msg/Switches.idl
// generated code does not contain a copyright notice

#ifndef RASPIMOUSE_MSGS__MSG__DETAIL__SWITCHES__TRAITS_HPP_
#define RASPIMOUSE_MSGS__MSG__DETAIL__SWITCHES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "raspimouse_msgs/msg/detail/switches__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace raspimouse_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Switches & msg,
  std::ostream & out)
{
  out << "{";
  // member: switch0
  {
    out << "switch0: ";
    rosidl_generator_traits::value_to_yaml(msg.switch0, out);
    out << ", ";
  }

  // member: switch1
  {
    out << "switch1: ";
    rosidl_generator_traits::value_to_yaml(msg.switch1, out);
    out << ", ";
  }

  // member: switch2
  {
    out << "switch2: ";
    rosidl_generator_traits::value_to_yaml(msg.switch2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Switches & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: switch0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "switch0: ";
    rosidl_generator_traits::value_to_yaml(msg.switch0, out);
    out << "\n";
  }

  // member: switch1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "switch1: ";
    rosidl_generator_traits::value_to_yaml(msg.switch1, out);
    out << "\n";
  }

  // member: switch2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "switch2: ";
    rosidl_generator_traits::value_to_yaml(msg.switch2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Switches & msg, bool use_flow_style = false)
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
  const raspimouse_msgs::msg::Switches & msg,
  std::ostream & out, size_t indentation = 0)
{
  raspimouse_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use raspimouse_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const raspimouse_msgs::msg::Switches & msg)
{
  return raspimouse_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<raspimouse_msgs::msg::Switches>()
{
  return "raspimouse_msgs::msg::Switches";
}

template<>
inline const char * name<raspimouse_msgs::msg::Switches>()
{
  return "raspimouse_msgs/msg/Switches";
}

template<>
struct has_fixed_size<raspimouse_msgs::msg::Switches>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<raspimouse_msgs::msg::Switches>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<raspimouse_msgs::msg::Switches>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RASPIMOUSE_MSGS__MSG__DETAIL__SWITCHES__TRAITS_HPP_
