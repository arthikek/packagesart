// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from raspimouse_msgs:msg/LightSensors.idl
// generated code does not contain a copyright notice

#ifndef RASPIMOUSE_MSGS__MSG__DETAIL__LIGHT_SENSORS__TRAITS_HPP_
#define RASPIMOUSE_MSGS__MSG__DETAIL__LIGHT_SENSORS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "raspimouse_msgs/msg/detail/light_sensors__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace raspimouse_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LightSensors & msg,
  std::ostream & out)
{
  out << "{";
  // member: forward_r
  {
    out << "forward_r: ";
    rosidl_generator_traits::value_to_yaml(msg.forward_r, out);
    out << ", ";
  }

  // member: forward_l
  {
    out << "forward_l: ";
    rosidl_generator_traits::value_to_yaml(msg.forward_l, out);
    out << ", ";
  }

  // member: left
  {
    out << "left: ";
    rosidl_generator_traits::value_to_yaml(msg.left, out);
    out << ", ";
  }

  // member: right
  {
    out << "right: ";
    rosidl_generator_traits::value_to_yaml(msg.right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LightSensors & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: forward_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "forward_r: ";
    rosidl_generator_traits::value_to_yaml(msg.forward_r, out);
    out << "\n";
  }

  // member: forward_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "forward_l: ";
    rosidl_generator_traits::value_to_yaml(msg.forward_l, out);
    out << "\n";
  }

  // member: left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left: ";
    rosidl_generator_traits::value_to_yaml(msg.left, out);
    out << "\n";
  }

  // member: right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right: ";
    rosidl_generator_traits::value_to_yaml(msg.right, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LightSensors & msg, bool use_flow_style = false)
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
  const raspimouse_msgs::msg::LightSensors & msg,
  std::ostream & out, size_t indentation = 0)
{
  raspimouse_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use raspimouse_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const raspimouse_msgs::msg::LightSensors & msg)
{
  return raspimouse_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<raspimouse_msgs::msg::LightSensors>()
{
  return "raspimouse_msgs::msg::LightSensors";
}

template<>
inline const char * name<raspimouse_msgs::msg::LightSensors>()
{
  return "raspimouse_msgs/msg/LightSensors";
}

template<>
struct has_fixed_size<raspimouse_msgs::msg::LightSensors>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<raspimouse_msgs::msg::LightSensors>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<raspimouse_msgs::msg::LightSensors>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RASPIMOUSE_MSGS__MSG__DETAIL__LIGHT_SENSORS__TRAITS_HPP_
