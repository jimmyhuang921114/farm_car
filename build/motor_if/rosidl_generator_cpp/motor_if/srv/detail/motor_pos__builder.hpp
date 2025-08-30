// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_if:srv/MotorPos.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_IF__SRV__DETAIL__MOTOR_POS__BUILDER_HPP_
#define MOTOR_IF__SRV__DETAIL__MOTOR_POS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_if/srv/detail/motor_pos__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_if
{

namespace srv
{

namespace builder
{

class Init_MotorPos_Request_angular
{
public:
  explicit Init_MotorPos_Request_angular(::motor_if::srv::MotorPos_Request & msg)
  : msg_(msg)
  {}
  ::motor_if::srv::MotorPos_Request angular(::motor_if::srv::MotorPos_Request::_angular_type arg)
  {
    msg_.angular = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_if::srv::MotorPos_Request msg_;
};

class Init_MotorPos_Request_linear
{
public:
  Init_MotorPos_Request_linear()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorPos_Request_angular linear(::motor_if::srv::MotorPos_Request::_linear_type arg)
  {
    msg_.linear = std::move(arg);
    return Init_MotorPos_Request_angular(msg_);
  }

private:
  ::motor_if::srv::MotorPos_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_if::srv::MotorPos_Request>()
{
  return motor_if::srv::builder::Init_MotorPos_Request_linear();
}

}  // namespace motor_if


namespace motor_if
{

namespace srv
{

namespace builder
{

class Init_MotorPos_Response_result
{
public:
  Init_MotorPos_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::motor_if::srv::MotorPos_Response result(::motor_if::srv::MotorPos_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_if::srv::MotorPos_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_if::srv::MotorPos_Response>()
{
  return motor_if::srv::builder::Init_MotorPos_Response_result();
}

}  // namespace motor_if

#endif  // MOTOR_IF__SRV__DETAIL__MOTOR_POS__BUILDER_HPP_
