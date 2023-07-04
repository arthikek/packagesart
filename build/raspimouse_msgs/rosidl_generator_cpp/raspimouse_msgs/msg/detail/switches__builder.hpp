// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from raspimouse_msgs:msg/Switches.idl
// generated code does not contain a copyright notice

#ifndef RASPIMOUSE_MSGS__MSG__DETAIL__SWITCHES__BUILDER_HPP_
#define RASPIMOUSE_MSGS__MSG__DETAIL__SWITCHES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "raspimouse_msgs/msg/detail/switches__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace raspimouse_msgs
{

namespace msg
{

namespace builder
{

class Init_Switches_switch2
{
public:
  explicit Init_Switches_switch2(::raspimouse_msgs::msg::Switches & msg)
  : msg_(msg)
  {}
  ::raspimouse_msgs::msg::Switches switch2(::raspimouse_msgs::msg::Switches::_switch2_type arg)
  {
    msg_.switch2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::raspimouse_msgs::msg::Switches msg_;
};

class Init_Switches_switch1
{
public:
  explicit Init_Switches_switch1(::raspimouse_msgs::msg::Switches & msg)
  : msg_(msg)
  {}
  Init_Switches_switch2 switch1(::raspimouse_msgs::msg::Switches::_switch1_type arg)
  {
    msg_.switch1 = std::move(arg);
    return Init_Switches_switch2(msg_);
  }

private:
  ::raspimouse_msgs::msg::Switches msg_;
};

class Init_Switches_switch0
{
public:
  Init_Switches_switch0()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Switches_switch1 switch0(::raspimouse_msgs::msg::Switches::_switch0_type arg)
  {
    msg_.switch0 = std::move(arg);
    return Init_Switches_switch1(msg_);
  }

private:
  ::raspimouse_msgs::msg::Switches msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::raspimouse_msgs::msg::Switches>()
{
  return raspimouse_msgs::msg::builder::Init_Switches_switch0();
}

}  // namespace raspimouse_msgs

#endif  // RASPIMOUSE_MSGS__MSG__DETAIL__SWITCHES__BUILDER_HPP_
