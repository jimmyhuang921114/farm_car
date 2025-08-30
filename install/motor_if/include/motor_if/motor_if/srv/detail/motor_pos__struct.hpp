// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_if:srv/MotorPos.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_IF__SRV__DETAIL__MOTOR_POS__STRUCT_HPP_
#define MOTOR_IF__SRV__DETAIL__MOTOR_POS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__motor_if__srv__MotorPos_Request __attribute__((deprecated))
#else
# define DEPRECATED__motor_if__srv__MotorPos_Request __declspec(deprecated)
#endif

namespace motor_if
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MotorPos_Request_
{
  using Type = MotorPos_Request_<ContainerAllocator>;

  explicit MotorPos_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear = 0.0f;
      this->angular = 0.0f;
    }
  }

  explicit MotorPos_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear = 0.0f;
      this->angular = 0.0f;
    }
  }

  // field types and members
  using _linear_type =
    float;
  _linear_type linear;
  using _angular_type =
    float;
  _angular_type angular;

  // setters for named parameter idiom
  Type & set__linear(
    const float & _arg)
  {
    this->linear = _arg;
    return *this;
  }
  Type & set__angular(
    const float & _arg)
  {
    this->angular = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_if::srv::MotorPos_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_if::srv::MotorPos_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_if::srv::MotorPos_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_if::srv::MotorPos_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_if::srv::MotorPos_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_if::srv::MotorPos_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_if::srv::MotorPos_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_if::srv::MotorPos_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_if::srv::MotorPos_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_if::srv::MotorPos_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_if__srv__MotorPos_Request
    std::shared_ptr<motor_if::srv::MotorPos_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_if__srv__MotorPos_Request
    std::shared_ptr<motor_if::srv::MotorPos_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorPos_Request_ & other) const
  {
    if (this->linear != other.linear) {
      return false;
    }
    if (this->angular != other.angular) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorPos_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorPos_Request_

// alias to use template instance with default allocator
using MotorPos_Request =
  motor_if::srv::MotorPos_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace motor_if


#ifndef _WIN32
# define DEPRECATED__motor_if__srv__MotorPos_Response __attribute__((deprecated))
#else
# define DEPRECATED__motor_if__srv__MotorPos_Response __declspec(deprecated)
#endif

namespace motor_if
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MotorPos_Response_
{
  using Type = MotorPos_Response_<ContainerAllocator>;

  explicit MotorPos_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0l;
    }
  }

  explicit MotorPos_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0l;
    }
  }

  // field types and members
  using _result_type =
    int32_t;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const int32_t & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_if::srv::MotorPos_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_if::srv::MotorPos_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_if::srv::MotorPos_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_if::srv::MotorPos_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_if::srv::MotorPos_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_if::srv::MotorPos_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_if::srv::MotorPos_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_if::srv::MotorPos_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_if::srv::MotorPos_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_if::srv::MotorPos_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_if__srv__MotorPos_Response
    std::shared_ptr<motor_if::srv::MotorPos_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_if__srv__MotorPos_Response
    std::shared_ptr<motor_if::srv::MotorPos_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorPos_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorPos_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorPos_Response_

// alias to use template instance with default allocator
using MotorPos_Response =
  motor_if::srv::MotorPos_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace motor_if

namespace motor_if
{

namespace srv
{

struct MotorPos
{
  using Request = motor_if::srv::MotorPos_Request;
  using Response = motor_if::srv::MotorPos_Response;
};

}  // namespace srv

}  // namespace motor_if

#endif  // MOTOR_IF__SRV__DETAIL__MOTOR_POS__STRUCT_HPP_
