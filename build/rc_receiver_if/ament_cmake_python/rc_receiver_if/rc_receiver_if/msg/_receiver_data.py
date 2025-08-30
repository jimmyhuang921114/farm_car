# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rc_receiver_if:msg/ReceiverData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'channel_data'
# Member 'wheel_control'
# Member 'twist'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ReceiverData(type):
    """Metaclass of message 'ReceiverData'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rc_receiver_if')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rc_receiver_if.msg.ReceiverData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__receiver_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__receiver_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__receiver_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__receiver_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__receiver_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CHANNEL_DATA__DEFAULT': numpy.array((1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, ), dtype=numpy.int16),
            'WHEEL_CONTROL__DEFAULT': numpy.array((0.0, 0.0, ), dtype=numpy.float32),
            'TWIST__DEFAULT': numpy.array((0.0, 0.0, ), dtype=numpy.float32),
        }

    @property
    def CHANNEL_DATA__DEFAULT(cls):
        """Return default value for message field 'channel_data'."""
        return numpy.array((1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, ), dtype=numpy.int16)

    @property
    def WHEEL_CONTROL__DEFAULT(cls):
        """Return default value for message field 'wheel_control'."""
        return numpy.array((0.0, 0.0, ), dtype=numpy.float32)

    @property
    def TWIST__DEFAULT(cls):
        """Return default value for message field 'twist'."""
        return numpy.array((0.0, 0.0, ), dtype=numpy.float32)


class ReceiverData(metaclass=Metaclass_ReceiverData):
    """Message class 'ReceiverData'."""

    __slots__ = [
        '_channel_data',
        '_wheel_control',
        '_twist',
    ]

    _fields_and_field_types = {
        'channel_data': 'int16[18]',
        'wheel_control': 'float[2]',
        'twist': 'float[2]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int16'), 18),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 2),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 2),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.channel_data = kwargs.get(
            'channel_data', ReceiverData.CHANNEL_DATA__DEFAULT)
        self.wheel_control = kwargs.get(
            'wheel_control', ReceiverData.WHEEL_CONTROL__DEFAULT)
        self.twist = kwargs.get(
            'twist', ReceiverData.TWIST__DEFAULT)

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if all(self.channel_data != other.channel_data):
            return False
        if all(self.wheel_control != other.wheel_control):
            return False
        if all(self.twist != other.twist):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def channel_data(self):
        """Message field 'channel_data'."""
        return self._channel_data

    @channel_data.setter
    def channel_data(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int16, \
                "The 'channel_data' numpy.ndarray() must have the dtype of 'numpy.int16'"
            assert value.size == 18, \
                "The 'channel_data' numpy.ndarray() must have a size of 18"
            self._channel_data = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 18 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'channel_data' field must be a set or sequence with length 18 and each value of type 'int' and each integer in [-32768, 32767]"
        self._channel_data = numpy.array(value, dtype=numpy.int16)

    @builtins.property
    def wheel_control(self):
        """Message field 'wheel_control'."""
        return self._wheel_control

    @wheel_control.setter
    def wheel_control(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'wheel_control' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 2, \
                "The 'wheel_control' numpy.ndarray() must have a size of 2"
            self._wheel_control = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 2 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'wheel_control' field must be a set or sequence with length 2 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._wheel_control = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def twist(self):
        """Message field 'twist'."""
        return self._twist

    @twist.setter
    def twist(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'twist' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 2, \
                "The 'twist' numpy.ndarray() must have a size of 2"
            self._twist = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 2 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'twist' field must be a set or sequence with length 2 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._twist = numpy.array(value, dtype=numpy.float32)
