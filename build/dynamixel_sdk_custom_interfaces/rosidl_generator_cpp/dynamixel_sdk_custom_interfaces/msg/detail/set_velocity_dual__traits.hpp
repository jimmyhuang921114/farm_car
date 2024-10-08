// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dynamixel_sdk_custom_interfaces:msg/SetVelocityDual.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY_DUAL__TRAITS_HPP_
#define DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY_DUAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dynamixel_sdk_custom_interfaces/msg/detail/set_velocity_dual__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dynamixel_sdk_custom_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SetVelocityDual & msg,
  std::ostream & out)
{
  out << "{";
  // member: motorspeed1
  {
    out << "motorspeed1: ";
    rosidl_generator_traits::value_to_yaml(msg.motorspeed1, out);
    out << ", ";
  }

  // member: motorspeed2
  {
    out << "motorspeed2: ";
    rosidl_generator_traits::value_to_yaml(msg.motorspeed2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetVelocityDual & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: motorspeed1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motorspeed1: ";
    rosidl_generator_traits::value_to_yaml(msg.motorspeed1, out);
    out << "\n";
  }

  // member: motorspeed2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motorspeed2: ";
    rosidl_generator_traits::value_to_yaml(msg.motorspeed2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetVelocityDual & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dynamixel_sdk_custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use dynamixel_sdk_custom_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dynamixel_sdk_custom_interfaces::msg::SetVelocityDual & msg,
  std::ostream & out, size_t indentation = 0)
{
  dynamixel_sdk_custom_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dynamixel_sdk_custom_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const dynamixel_sdk_custom_interfaces::msg::SetVelocityDual & msg)
{
  return dynamixel_sdk_custom_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dynamixel_sdk_custom_interfaces::msg::SetVelocityDual>()
{
  return "dynamixel_sdk_custom_interfaces::msg::SetVelocityDual";
}

template<>
inline const char * name<dynamixel_sdk_custom_interfaces::msg::SetVelocityDual>()
{
  return "dynamixel_sdk_custom_interfaces/msg/SetVelocityDual";
}

template<>
struct has_fixed_size<dynamixel_sdk_custom_interfaces::msg::SetVelocityDual>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dynamixel_sdk_custom_interfaces::msg::SetVelocityDual>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dynamixel_sdk_custom_interfaces::msg::SetVelocityDual>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY_DUAL__TRAITS_HPP_
