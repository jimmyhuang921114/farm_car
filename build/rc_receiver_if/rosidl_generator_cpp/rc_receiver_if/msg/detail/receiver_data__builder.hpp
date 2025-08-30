// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rc_receiver_if:msg/ReceiverData.idl
// generated code does not contain a copyright notice

#ifndef RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__BUILDER_HPP_
#define RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rc_receiver_if/msg/detail/receiver_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rc_receiver_if
{

namespace msg
{

namespace builder
{

class Init_ReceiverData_twist
{
public:
  explicit Init_ReceiverData_twist(::rc_receiver_if::msg::ReceiverData & msg)
  : msg_(msg)
  {}
  ::rc_receiver_if::msg::ReceiverData twist(::rc_receiver_if::msg::ReceiverData::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rc_receiver_if::msg::ReceiverData msg_;
};

class Init_ReceiverData_wheel_control
{
public:
  explicit Init_ReceiverData_wheel_control(::rc_receiver_if::msg::ReceiverData & msg)
  : msg_(msg)
  {}
  Init_ReceiverData_twist wheel_control(::rc_receiver_if::msg::ReceiverData::_wheel_control_type arg)
  {
    msg_.wheel_control = std::move(arg);
    return Init_ReceiverData_twist(msg_);
  }

private:
  ::rc_receiver_if::msg::ReceiverData msg_;
};

class Init_ReceiverData_channel_data
{
public:
  Init_ReceiverData_channel_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReceiverData_wheel_control channel_data(::rc_receiver_if::msg::ReceiverData::_channel_data_type arg)
  {
    msg_.channel_data = std::move(arg);
    return Init_ReceiverData_wheel_control(msg_);
  }

private:
  ::rc_receiver_if::msg::ReceiverData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rc_receiver_if::msg::ReceiverData>()
{
  return rc_receiver_if::msg::builder::Init_ReceiverData_channel_data();
}

}  // namespace rc_receiver_if

#endif  // RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__BUILDER_HPP_
