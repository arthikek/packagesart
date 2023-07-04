// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from raspimouse_msgs:msg/LightSensors.idl
// generated code does not contain a copyright notice

#ifndef RASPIMOUSE_MSGS__MSG__DETAIL__LIGHT_SENSORS__BUILDER_HPP_
#define RASPIMOUSE_MSGS__MSG__DETAIL__LIGHT_SENSORS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "raspimouse_msgs/msg/detail/light_sensors__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace raspimouse_msgs
{

namespace msg
{

namespace builder
{

class Init_LightSensors_right
{
public:
  explicit Init_LightSensors_right(::raspimouse_msgs::msg::LightSensors & msg)
  : msg_(msg)
  {}
  ::raspimouse_msgs::msg::LightSensors right(::raspimouse_msgs::msg::LightSensors::_right_type arg)
  {
    msg_.right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::raspimouse_msgs::msg::LightSensors msg_;
};

class Init_LightSensors_left
{
public:
  explicit Init_LightSensors_left(::raspimouse_msgs::msg::LightSensors & msg)
  : msg_(msg)
  {}
  Init_LightSensors_right left(::raspimouse_msgs::msg::LightSensors::_left_type arg)
  {
    msg_.left = std::move(arg);
    return Init_LightSensors_right(msg_);
  }

private:
  ::raspimouse_msgs::msg::LightSensors msg_;
};

class Init_LightSensors_forward_l
{
public:
  explicit Init_LightSensors_forward_l(::raspimouse_msgs::msg::LightSensors & msg)
  : msg_(msg)
  {}
  Init_LightSensors_left forward_l(::raspimouse_msgs::msg::LightSensors::_forward_l_type arg)
  {
    msg_.forward_l = std::move(arg);
    return Init_LightSensors_left(msg_);
  }

private:
  ::raspimouse_msgs::msg::LightSensors msg_;
};

class Init_LightSensors_forward_r
{
public:
  Init_LightSensors_forward_r()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LightSensors_forward_l forward_r(::raspimouse_msgs::msg::LightSensors::_forward_r_type arg)
  {
    msg_.forward_r = std::move(arg);
    return Init_LightSensors_forward_l(msg_);
  }

private:
  ::raspimouse_msgs::msg::LightSensors msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::raspimouse_msgs::msg::LightSensors>()
{
  return raspimouse_msgs::msg::builder::Init_LightSensors_forward_r();
}

}  // namespace raspimouse_msgs

#endif  // RASPIMOUSE_MSGS__MSG__DETAIL__LIGHT_SENSORS__BUILDER_HPP_
