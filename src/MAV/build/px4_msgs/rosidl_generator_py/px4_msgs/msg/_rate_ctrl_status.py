# generated from rosidl_generator_py/resource/_idl.py.em
# with input from px4_msgs:msg/RateCtrlStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RateCtrlStatus(type):
    """Metaclass of message 'RateCtrlStatus'."""

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
            module = import_type_support('px4_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'px4_msgs.msg.RateCtrlStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__rate_ctrl_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__rate_ctrl_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__rate_ctrl_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__rate_ctrl_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__rate_ctrl_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RateCtrlStatus(metaclass=Metaclass_RateCtrlStatus):
    """Message class 'RateCtrlStatus'."""

    __slots__ = [
        '_timestamp',
        '_rollspeed_integ',
        '_pitchspeed_integ',
        '_yawspeed_integ',
    ]

    _fields_and_field_types = {
        'timestamp': 'uint64',
        'rollspeed_integ': 'float',
        'pitchspeed_integ': 'float',
        'yawspeed_integ': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.timestamp = kwargs.get('timestamp', int())
        self.rollspeed_integ = kwargs.get('rollspeed_integ', float())
        self.pitchspeed_integ = kwargs.get('pitchspeed_integ', float())
        self.yawspeed_integ = kwargs.get('yawspeed_integ', float())

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
        if self.timestamp != other.timestamp:
            return False
        if self.rollspeed_integ != other.rollspeed_integ:
            return False
        if self.pitchspeed_integ != other.pitchspeed_integ:
            return False
        if self.yawspeed_integ != other.yawspeed_integ:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'timestamp' field must be an unsigned integer in [0, 18446744073709551615]"
        self._timestamp = value

    @builtins.property
    def rollspeed_integ(self):
        """Message field 'rollspeed_integ'."""
        return self._rollspeed_integ

    @rollspeed_integ.setter
    def rollspeed_integ(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rollspeed_integ' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'rollspeed_integ' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._rollspeed_integ = value

    @builtins.property
    def pitchspeed_integ(self):
        """Message field 'pitchspeed_integ'."""
        return self._pitchspeed_integ

    @pitchspeed_integ.setter
    def pitchspeed_integ(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitchspeed_integ' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitchspeed_integ' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitchspeed_integ = value

    @builtins.property
    def yawspeed_integ(self):
        """Message field 'yawspeed_integ'."""
        return self._yawspeed_integ

    @yawspeed_integ.setter
    def yawspeed_integ(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yawspeed_integ' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yawspeed_integ' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yawspeed_integ = value
