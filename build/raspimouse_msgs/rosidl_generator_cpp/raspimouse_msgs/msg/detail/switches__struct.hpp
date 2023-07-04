// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from raspimouse_msgs:msg/Switches.idl
// generated code does not contain a copyright notice

#ifndef RASPIMOUSE_MSGS__MSG__DETAIL__SWITCHES__STRUCT_HPP_
#define RASPIMOUSE_MSGS__MSG__DETAIL__SWITCHES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__raspimouse_msgs__msg__Switches __attribute__((deprecated))
#else
# define DEPRECATED__raspimouse_msgs__msg__Switches __declspec(deprecated)
#endif

namespace raspimouse_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Switches_
{
  using Type = Switches_<ContainerAllocator>;

  explicit Switches_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->switch0 = false;
      this->switch1 = false;
      this->switch2 = false;
    }
  }

  explicit Switches_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->switch0 = false;
      this->switch1 = false;
      this->switch2 = false;
    }
  }

  // field types and members
  using _switch0_type =
    bool;
  _switch0_type switch0;
  using _switch1_type =
    bool;
  _switch1_type switch1;
  using _switch2_type =
    bool;
  _switch2_type switch2;

  // setters for named parameter idiom
  Type & set__switch0(
    const bool & _arg)
  {
    this->switch0 = _arg;
    return *this;
  }
  Type & set__switch1(
    const bool & _arg)
  {
    this->switch1 = _arg;
    return *this;
  }
  Type & set__switch2(
    const bool & _arg)
  {
    this->switch2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    raspimouse_msgs::msg::Switches_<ContainerAllocator> *;
  using ConstRawPtr =
    const raspimouse_msgs::msg::Switches_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<raspimouse_msgs::msg::Switches_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<raspimouse_msgs::msg::Switches_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      raspimouse_msgs::msg::Switches_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<raspimouse_msgs::msg::Switches_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      raspimouse_msgs::msg::Switches_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<raspimouse_msgs::msg::Switches_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<raspimouse_msgs::msg::Switches_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<raspimouse_msgs::msg::Switches_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__raspimouse_msgs__msg__Switches
    std::shared_ptr<raspimouse_msgs::msg::Switches_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__raspimouse_msgs__msg__Switches
    std::shared_ptr<raspimouse_msgs::msg::Switches_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Switches_ & other) const
  {
    if (this->switch0 != other.switch0) {
      return false;
    }
    if (this->switch1 != other.switch1) {
      return false;
    }
    if (this->switch2 != other.switch2) {
      return false;
    }
    return true;
  }
  bool operator!=(const Switches_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Switches_

// alias to use template instance with default allocator
using Switches =
  raspimouse_msgs::msg::Switches_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace raspimouse_msgs

#endif  // RASPIMOUSE_MSGS__MSG__DETAIL__SWITCHES__STRUCT_HPP_
