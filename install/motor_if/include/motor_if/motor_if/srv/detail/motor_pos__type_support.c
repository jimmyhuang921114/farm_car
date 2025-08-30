// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from motor_if:srv/MotorPos.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "motor_if/srv/detail/motor_pos__rosidl_typesupport_introspection_c.h"
#include "motor_if/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "motor_if/srv/detail/motor_pos__functions.h"
#include "motor_if/srv/detail/motor_pos__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void motor_if__srv__MotorPos_Request__rosidl_typesupport_introspection_c__MotorPos_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  motor_if__srv__MotorPos_Request__init(message_memory);
}

void motor_if__srv__MotorPos_Request__rosidl_typesupport_introspection_c__MotorPos_Request_fini_function(void * message_memory)
{
  motor_if__srv__MotorPos_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember motor_if__srv__MotorPos_Request__rosidl_typesupport_introspection_c__MotorPos_Request_message_member_array[2] = {
  {
    "linear",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_if__srv__MotorPos_Request, linear),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angular",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_if__srv__MotorPos_Request, angular),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers motor_if__srv__MotorPos_Request__rosidl_typesupport_introspection_c__MotorPos_Request_message_members = {
  "motor_if__srv",  // message namespace
  "MotorPos_Request",  // message name
  2,  // number of fields
  sizeof(motor_if__srv__MotorPos_Request),
  motor_if__srv__MotorPos_Request__rosidl_typesupport_introspection_c__MotorPos_Request_message_member_array,  // message members
  motor_if__srv__MotorPos_Request__rosidl_typesupport_introspection_c__MotorPos_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  motor_if__srv__MotorPos_Request__rosidl_typesupport_introspection_c__MotorPos_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t motor_if__srv__MotorPos_Request__rosidl_typesupport_introspection_c__MotorPos_Request_message_type_support_handle = {
  0,
  &motor_if__srv__MotorPos_Request__rosidl_typesupport_introspection_c__MotorPos_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motor_if
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_if, srv, MotorPos_Request)() {
  if (!motor_if__srv__MotorPos_Request__rosidl_typesupport_introspection_c__MotorPos_Request_message_type_support_handle.typesupport_identifier) {
    motor_if__srv__MotorPos_Request__rosidl_typesupport_introspection_c__MotorPos_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &motor_if__srv__MotorPos_Request__rosidl_typesupport_introspection_c__MotorPos_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "motor_if/srv/detail/motor_pos__rosidl_typesupport_introspection_c.h"
// already included above
// #include "motor_if/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "motor_if/srv/detail/motor_pos__functions.h"
// already included above
// #include "motor_if/srv/detail/motor_pos__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void motor_if__srv__MotorPos_Response__rosidl_typesupport_introspection_c__MotorPos_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  motor_if__srv__MotorPos_Response__init(message_memory);
}

void motor_if__srv__MotorPos_Response__rosidl_typesupport_introspection_c__MotorPos_Response_fini_function(void * message_memory)
{
  motor_if__srv__MotorPos_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember motor_if__srv__MotorPos_Response__rosidl_typesupport_introspection_c__MotorPos_Response_message_member_array[1] = {
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_if__srv__MotorPos_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers motor_if__srv__MotorPos_Response__rosidl_typesupport_introspection_c__MotorPos_Response_message_members = {
  "motor_if__srv",  // message namespace
  "MotorPos_Response",  // message name
  1,  // number of fields
  sizeof(motor_if__srv__MotorPos_Response),
  motor_if__srv__MotorPos_Response__rosidl_typesupport_introspection_c__MotorPos_Response_message_member_array,  // message members
  motor_if__srv__MotorPos_Response__rosidl_typesupport_introspection_c__MotorPos_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  motor_if__srv__MotorPos_Response__rosidl_typesupport_introspection_c__MotorPos_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t motor_if__srv__MotorPos_Response__rosidl_typesupport_introspection_c__MotorPos_Response_message_type_support_handle = {
  0,
  &motor_if__srv__MotorPos_Response__rosidl_typesupport_introspection_c__MotorPos_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motor_if
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_if, srv, MotorPos_Response)() {
  if (!motor_if__srv__MotorPos_Response__rosidl_typesupport_introspection_c__MotorPos_Response_message_type_support_handle.typesupport_identifier) {
    motor_if__srv__MotorPos_Response__rosidl_typesupport_introspection_c__MotorPos_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &motor_if__srv__MotorPos_Response__rosidl_typesupport_introspection_c__MotorPos_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "motor_if/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "motor_if/srv/detail/motor_pos__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers motor_if__srv__detail__motor_pos__rosidl_typesupport_introspection_c__MotorPos_service_members = {
  "motor_if__srv",  // service namespace
  "MotorPos",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // motor_if__srv__detail__motor_pos__rosidl_typesupport_introspection_c__MotorPos_Request_message_type_support_handle,
  NULL  // response message
  // motor_if__srv__detail__motor_pos__rosidl_typesupport_introspection_c__MotorPos_Response_message_type_support_handle
};

static rosidl_service_type_support_t motor_if__srv__detail__motor_pos__rosidl_typesupport_introspection_c__MotorPos_service_type_support_handle = {
  0,
  &motor_if__srv__detail__motor_pos__rosidl_typesupport_introspection_c__MotorPos_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_if, srv, MotorPos_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_if, srv, MotorPos_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motor_if
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_if, srv, MotorPos)() {
  if (!motor_if__srv__detail__motor_pos__rosidl_typesupport_introspection_c__MotorPos_service_type_support_handle.typesupport_identifier) {
    motor_if__srv__detail__motor_pos__rosidl_typesupport_introspection_c__MotorPos_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)motor_if__srv__detail__motor_pos__rosidl_typesupport_introspection_c__MotorPos_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_if, srv, MotorPos_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_if, srv, MotorPos_Response)()->data;
  }

  return &motor_if__srv__detail__motor_pos__rosidl_typesupport_introspection_c__MotorPos_service_type_support_handle;
}
