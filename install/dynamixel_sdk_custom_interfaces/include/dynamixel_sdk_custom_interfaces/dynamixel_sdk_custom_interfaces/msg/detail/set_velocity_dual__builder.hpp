// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamixel_sdk_custom_interfaces:msg/SetVelocityDual.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY_DUAL__BUILDER_HPP_
#define DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY_DUAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamixel_sdk_custom_interfaces/msg/detail/set_velocity_dual__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamixel_sdk_custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_SetVelocityDual_motorspeed2
{
public:
  explicit Init_SetVelocityDual_motorspeed2(::dynamixel_sdk_custom_interfaces::msg::SetVelocityDual & msg)
  : msg_(msg)
  {}
  ::dynamixel_sdk_custom_interfaces::msg::SetVelocityDual motorspeed2(::dynamixel_sdk_custom_interfaces::msg::SetVelocityDual::_motorspeed2_type arg)
  {
    msg_.motorspeed2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_sdk_custom_interfaces::msg::SetVelocityDual msg_;
};

class Init_SetVelocityDual_motorspeed1
{
public:
  Init_SetVelocityDual_motorspeed1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetVelocityDual_motorspeed2 motorspeed1(::dynamixel_sdk_custom_interfaces::msg::SetVelocityDual::_motorspeed1_type arg)
  {
    msg_.motorspeed1 = std::move(arg);
    return Init_SetVelocityDual_motorspeed2(msg_);
  }

private:
  ::dynamixel_sdk_custom_interfaces::msg::SetVelocityDual msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_sdk_custom_interfaces::msg::SetVelocityDual>()
{
  return dynamixel_sdk_custom_interfaces::msg::builder::Init_SetVelocityDual_motorspeed1();
}

}  // namespace dynamixel_sdk_custom_interfaces

#endif  // DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY_DUAL__BUILDER_HPP_
