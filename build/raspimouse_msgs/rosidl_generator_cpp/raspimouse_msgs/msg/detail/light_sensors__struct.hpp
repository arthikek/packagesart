// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from raspimouse_msgs:msg/LightSensors.idl
// generated code does not contain a copyright notice

#ifndef RASPIMOUSE_MSGS__MSG__DETAIL__LIGHT_SENSORS__STRUCT_HPP_
#define RASPIMOUSE_MSGS__MSG__DETAIL__LIGHT_SENSORS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__raspimouse_msgs__msg__LightSensors __attribute__((deprecated))
#else
# define DEPRECATED__raspimouse_msgs__msg__LightSensors __declspec(deprecated)
#endif

namespace raspimouse_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LightSensors_
{
  using Type = LightSensors_<ContainerAllocator>;

  explicit LightSensors_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->forward_r = 0;
      this->forward_l = 0;
      this->left = 0;
      this->right = 0;
    }
  }

  explicit LightSensors_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->forward_r = 0;
      this->forward_l = 0;
      this->left = 0;
      this->right = 0;
    }
  }

  // field types and members
  using _forward_r_type =
    int16_t;
  _forward_r_type forward_r;
  using _forward_l_type =
    int16_t;
  _forward_l_type forward_l;
  using _left_type =
    int16_t;
  _left_type left;
  using _right_type =
    int16_t;
  _right_type right;

  // setters for named parameter idiom
  Type & set__forward_r(
    const int16_t & _arg)
  {
    this->forward_r = _arg;
    return *this;
  }
  Type & set__forward_l(
    const int16_t & _arg)
  {
    this->forward_l = _arg;
    return *this;
  }
  Type & set__left(
    const int16_t & _arg)
  {
    this->left = _arg;
    return *this;
  }
  Type & set__right(
    const int16_t & _arg)
  {
    this->right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    raspimouse_msgs::msg::LightSensors_<ContainerAllocator> *;
  using ConstRawPtr =
    const raspimouse_msgs::msg::LightSensors_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<raspimouse_msgs::msg::LightSensors_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<raspimouse_msgs::msg::LightSensors_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      raspimouse_msgs::msg::LightSensors_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<raspimouse_msgs::msg::LightSensors_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      raspimouse_msgs::msg::LightSensors_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<raspimouse_msgs::msg::LightSensors_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<raspimouse_msgs::msg::LightSensors_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<raspimouse_msgs::msg::LightSensors_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__raspimouse_msgs__msg__LightSensors
    std::shared_ptr<raspimouse_msgs::msg::LightSensors_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__raspimouse_msgs__msg__LightSensors
    std::shared_ptr<raspimouse_msgs::msg::LightSensors_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LightSensors_ & other) const
  {
    if (this->forward_r != other.forward_r) {
      return false;
    }
    if (this->forward_l != other.forward_l) {
      return false;
    }
    if (this->left != other.left) {
      return false;
    }
    if (this->right != other.right) {
      return false;
    }
    return true;
  }
  bool operator!=(const LightSensors_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LightSensors_

// alias to use template instance with default allocator
using LightSensors =
  raspimouse_msgs::msg::LightSensors_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace raspimouse_msgs

#endif  // RASPIMOUSE_MSGS__MSG__DETAIL__LIGHT_SENSORS__STRUCT_HPP_
