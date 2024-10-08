// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dynamixel_sdk_custom_interfaces:msg/SetVelocityDual.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY_DUAL__STRUCT_HPP_
#define DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY_DUAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dynamixel_sdk_custom_interfaces__msg__SetVelocityDual __attribute__((deprecated))
#else
# define DEPRECATED__dynamixel_sdk_custom_interfaces__msg__SetVelocityDual __declspec(deprecated)
#endif

namespace dynamixel_sdk_custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SetVelocityDual_
{
  using Type = SetVelocityDual_<ContainerAllocator>;

  explicit SetVelocityDual_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motorspeed1 = 0l;
      this->motorspeed2 = 0l;
    }
  }

  explicit SetVelocityDual_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motorspeed1 = 0l;
      this->motorspeed2 = 0l;
    }
  }

  // field types and members
  using _motorspeed1_type =
    int32_t;
  _motorspeed1_type motorspeed1;
  using _motorspeed2_type =
    int32_t;
  _motorspeed2_type motorspeed2;

  // setters for named parameter idiom
  Type & set__motorspeed1(
    const int32_t & _arg)
  {
    this->motorspeed1 = _arg;
    return *this;
  }
  Type & set__motorspeed2(
    const int32_t & _arg)
  {
    this->motorspeed2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dynamixel_sdk_custom_interfaces::msg::SetVelocityDual_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamixel_sdk_custom_interfaces::msg::SetVelocityDual_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamixel_sdk_custom_interfaces::msg::SetVelocityDual_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamixel_sdk_custom_interfaces::msg::SetVelocityDual_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamixel_sdk_custom_interfaces::msg::SetVelocityDual_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamixel_sdk_custom_interfaces::msg::SetVelocityDual_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamixel_sdk_custom_interfaces::msg::SetVelocityDual_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamixel_sdk_custom_interfaces::msg::SetVelocityDual_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamixel_sdk_custom_interfaces::msg::SetVelocityDual_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamixel_sdk_custom_interfaces::msg::SetVelocityDual_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamixel_sdk_custom_interfaces__msg__SetVelocityDual
    std::shared_ptr<dynamixel_sdk_custom_interfaces::msg::SetVelocityDual_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamixel_sdk_custom_interfaces__msg__SetVelocityDual
    std::shared_ptr<dynamixel_sdk_custom_interfaces::msg::SetVelocityDual_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetVelocityDual_ & other) const
  {
    if (this->motorspeed1 != other.motorspeed1) {
      return false;
    }
    if (this->motorspeed2 != other.motorspeed2) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetVelocityDual_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetVelocityDual_

// alias to use template instance with default allocator
using SetVelocityDual =
  dynamixel_sdk_custom_interfaces::msg::SetVelocityDual_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dynamixel_sdk_custom_interfaces

#endif  // DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY_DUAL__STRUCT_HPP_
