// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rc_receiver_if:msg/ReceiverData.idl
// generated code does not contain a copyright notice

#ifndef RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__STRUCT_HPP_
#define RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rc_receiver_if__msg__ReceiverData __attribute__((deprecated))
#else
# define DEPRECATED__rc_receiver_if__msg__ReceiverData __declspec(deprecated)
#endif

namespace rc_receiver_if
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ReceiverData_
{
  using Type = ReceiverData_<ContainerAllocator>;

  explicit ReceiverData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      std::fill<typename std::array<int16_t, 18>::iterator, int16_t>(this->channel_data.begin(), this->channel_data.end(), 1000);
      std::fill<typename std::array<float, 2>::iterator, float>(this->wheel_control.begin(), this->wheel_control.end(), 0.0f);
      std::fill<typename std::array<float, 2>::iterator, float>(this->twist.begin(), this->twist.end(), 0.0f);
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      std::fill<typename std::array<int16_t, 18>::iterator, int16_t>(this->channel_data.begin(), this->channel_data.end(), 0);
      std::fill<typename std::array<float, 2>::iterator, float>(this->wheel_control.begin(), this->wheel_control.end(), 0.0f);
      std::fill<typename std::array<float, 2>::iterator, float>(this->twist.begin(), this->twist.end(), 0.0f);
    }
  }

  explicit ReceiverData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : channel_data(_alloc),
    wheel_control(_alloc),
    twist(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      std::fill<typename std::array<int16_t, 18>::iterator, int16_t>(this->channel_data.begin(), this->channel_data.end(), 1000);
      std::fill<typename std::array<float, 2>::iterator, float>(this->wheel_control.begin(), this->wheel_control.end(), 0.0f);
      std::fill<typename std::array<float, 2>::iterator, float>(this->twist.begin(), this->twist.end(), 0.0f);
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      std::fill<typename std::array<int16_t, 18>::iterator, int16_t>(this->channel_data.begin(), this->channel_data.end(), 0);
      std::fill<typename std::array<float, 2>::iterator, float>(this->wheel_control.begin(), this->wheel_control.end(), 0.0f);
      std::fill<typename std::array<float, 2>::iterator, float>(this->twist.begin(), this->twist.end(), 0.0f);
    }
  }

  // field types and members
  using _channel_data_type =
    std::array<int16_t, 18>;
  _channel_data_type channel_data;
  using _wheel_control_type =
    std::array<float, 2>;
  _wheel_control_type wheel_control;
  using _twist_type =
    std::array<float, 2>;
  _twist_type twist;

  // setters for named parameter idiom
  Type & set__channel_data(
    const std::array<int16_t, 18> & _arg)
  {
    this->channel_data = _arg;
    return *this;
  }
  Type & set__wheel_control(
    const std::array<float, 2> & _arg)
  {
    this->wheel_control = _arg;
    return *this;
  }
  Type & set__twist(
    const std::array<float, 2> & _arg)
  {
    this->twist = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rc_receiver_if::msg::ReceiverData_<ContainerAllocator> *;
  using ConstRawPtr =
    const rc_receiver_if::msg::ReceiverData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rc_receiver_if::msg::ReceiverData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rc_receiver_if::msg::ReceiverData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rc_receiver_if::msg::ReceiverData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rc_receiver_if::msg::ReceiverData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rc_receiver_if::msg::ReceiverData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rc_receiver_if::msg::ReceiverData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rc_receiver_if::msg::ReceiverData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rc_receiver_if::msg::ReceiverData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rc_receiver_if__msg__ReceiverData
    std::shared_ptr<rc_receiver_if::msg::ReceiverData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rc_receiver_if__msg__ReceiverData
    std::shared_ptr<rc_receiver_if::msg::ReceiverData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ReceiverData_ & other) const
  {
    if (this->channel_data != other.channel_data) {
      return false;
    }
    if (this->wheel_control != other.wheel_control) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    return true;
  }
  bool operator!=(const ReceiverData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ReceiverData_

// alias to use template instance with default allocator
using ReceiverData =
  rc_receiver_if::msg::ReceiverData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rc_receiver_if

#endif  // RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__STRUCT_HPP_
