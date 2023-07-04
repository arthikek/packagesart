// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from raspimouse_msgs:msg/Leds.idl
// generated code does not contain a copyright notice

#ifndef RASPIMOUSE_MSGS__MSG__DETAIL__LEDS__STRUCT_HPP_
#define RASPIMOUSE_MSGS__MSG__DETAIL__LEDS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__raspimouse_msgs__msg__Leds __attribute__((deprecated))
#else
# define DEPRECATED__raspimouse_msgs__msg__Leds __declspec(deprecated)
#endif

namespace raspimouse_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Leds_
{
  using Type = Leds_<ContainerAllocator>;

  explicit Leds_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->led0 = false;
      this->led1 = false;
      this->led2 = false;
      this->led3 = false;
    }
  }

  explicit Leds_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->led0 = false;
      this->led1 = false;
      this->led2 = false;
      this->led3 = false;
    }
  }

  // field types and members
  using _led0_type =
    bool;
  _led0_type led0;
  using _led1_type =
    bool;
  _led1_type led1;
  using _led2_type =
    bool;
  _led2_type led2;
  using _led3_type =
    bool;
  _led3_type led3;

  // setters for named parameter idiom
  Type & set__led0(
    const bool & _arg)
  {
    this->led0 = _arg;
    return *this;
  }
  Type & set__led1(
    const bool & _arg)
  {
    this->led1 = _arg;
    return *this;
  }
  Type & set__led2(
    const bool & _arg)
  {
    this->led2 = _arg;
    return *this;
  }
  Type & set__led3(
    const bool & _arg)
  {
    this->led3 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    raspimouse_msgs::msg::Leds_<ContainerAllocator> *;
  using ConstRawPtr =
    const raspimouse_msgs::msg::Leds_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<raspimouse_msgs::msg::Leds_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<raspimouse_msgs::msg::Leds_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      raspimouse_msgs::msg::Leds_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<raspimouse_msgs::msg::Leds_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      raspimouse_msgs::msg::Leds_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<raspimouse_msgs::msg::Leds_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<raspimouse_msgs::msg::Leds_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<raspimouse_msgs::msg::Leds_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__raspimouse_msgs__msg__Leds
    std::shared_ptr<raspimouse_msgs::msg::Leds_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__raspimouse_msgs__msg__Leds
    std::shared_ptr<raspimouse_msgs::msg::Leds_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Leds_ & other) const
  {
    if (this->led0 != other.led0) {
      return false;
    }
    if (this->led1 != other.led1) {
      return false;
    }
    if (this->led2 != other.led2) {
      return false;
    }
    if (this->led3 != other.led3) {
      return false;
    }
    return true;
  }
  bool operator!=(const Leds_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Leds_

// alias to use template instance with default allocator
using Leds =
  raspimouse_msgs::msg::Leds_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace raspimouse_msgs

#endif  // RASPIMOUSE_MSGS__MSG__DETAIL__LEDS__STRUCT_HPP_
