// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rc_receiver_if:msg/ReceiverData.idl
// generated code does not contain a copyright notice

#ifndef RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__TRAITS_HPP_
#define RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rc_receiver_if/msg/detail/receiver_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rc_receiver_if
{

namespace msg
{

inline void to_flow_style_yaml(
  const ReceiverData & msg,
  std::ostream & out)
{
  out << "{";
  // member: channel_data
  {
    if (msg.channel_data.size() == 0) {
      out << "channel_data: []";
    } else {
      out << "channel_data: [";
      size_t pending_items = msg.channel_data.size();
      for (auto item : msg.channel_data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: wheel_control
  {
    if (msg.wheel_control.size() == 0) {
      out << "wheel_control: []";
    } else {
      out << "wheel_control: [";
      size_t pending_items = msg.wheel_control.size();
      for (auto item : msg.wheel_control) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: twist
  {
    if (msg.twist.size() == 0) {
      out << "twist: []";
    } else {
      out << "twist: [";
      size_t pending_items = msg.twist.size();
      for (auto item : msg.twist) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ReceiverData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: channel_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.channel_data.size() == 0) {
      out << "channel_data: []\n";
    } else {
      out << "channel_data:\n";
      for (auto item : msg.channel_data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: wheel_control
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.wheel_control.size() == 0) {
      out << "wheel_control: []\n";
    } else {
      out << "wheel_control:\n";
      for (auto item : msg.wheel_control) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.twist.size() == 0) {
      out << "twist: []\n";
    } else {
      out << "twist:\n";
      for (auto item : msg.twist) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ReceiverData & msg, bool use_flow_style = false)
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

}  // namespace rc_receiver_if

namespace rosidl_generator_traits
{

[[deprecated("use rc_receiver_if::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rc_receiver_if::msg::ReceiverData & msg,
  std::ostream & out, size_t indentation = 0)
{
  rc_receiver_if::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rc_receiver_if::msg::to_yaml() instead")]]
inline std::string to_yaml(const rc_receiver_if::msg::ReceiverData & msg)
{
  return rc_receiver_if::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rc_receiver_if::msg::ReceiverData>()
{
  return "rc_receiver_if::msg::ReceiverData";
}

template<>
inline const char * name<rc_receiver_if::msg::ReceiverData>()
{
  return "rc_receiver_if/msg/ReceiverData";
}

template<>
struct has_fixed_size<rc_receiver_if::msg::ReceiverData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rc_receiver_if::msg::ReceiverData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rc_receiver_if::msg::ReceiverData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RC_RECEIVER_IF__MSG__DETAIL__RECEIVER_DATA__TRAITS_HPP_
