// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from raspimouse_msgs:msg/Leds.idl
// generated code does not contain a copyright notice

#ifndef RASPIMOUSE_MSGS__MSG__DETAIL__LEDS__BUILDER_HPP_
#define RASPIMOUSE_MSGS__MSG__DETAIL__LEDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "raspimouse_msgs/msg/detail/leds__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace raspimouse_msgs
{

namespace msg
{

namespace builder
{

class Init_Leds_led3
{
public:
  explicit Init_Leds_led3(::raspimouse_msgs::msg::Leds & msg)
  : msg_(msg)
  {}
  ::raspimouse_msgs::msg::Leds led3(::raspimouse_msgs::msg::Leds::_led3_type arg)
  {
    msg_.led3 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::raspimouse_msgs::msg::Leds msg_;
};

class Init_Leds_led2
{
public:
  explicit Init_Leds_led2(::raspimouse_msgs::msg::Leds & msg)
  : msg_(msg)
  {}
  Init_Leds_led3 led2(::raspimouse_msgs::msg::Leds::_led2_type arg)
  {
    msg_.led2 = std::move(arg);
    return Init_Leds_led3(msg_);
  }

private:
  ::raspimouse_msgs::msg::Leds msg_;
};

class Init_Leds_led1
{
public:
  explicit Init_Leds_led1(::raspimouse_msgs::msg::Leds & msg)
  : msg_(msg)
  {}
  Init_Leds_led2 led1(::raspimouse_msgs::msg::Leds::_led1_type arg)
  {
    msg_.led1 = std::move(arg);
    return Init_Leds_led2(msg_);
  }

private:
  ::raspimouse_msgs::msg::Leds msg_;
};

class Init_Leds_led0
{
public:
  Init_Leds_led0()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Leds_led1 led0(::raspimouse_msgs::msg::Leds::_led0_type arg)
  {
    msg_.led0 = std::move(arg);
    return Init_Leds_led1(msg_);
  }

private:
  ::raspimouse_msgs::msg::Leds msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::raspimouse_msgs::msg::Leds>()
{
  return raspimouse_msgs::msg::builder::Init_Leds_led0();
}

}  // namespace raspimouse_msgs

#endif  // RASPIMOUSE_MSGS__MSG__DETAIL__LEDS__BUILDER_HPP_
