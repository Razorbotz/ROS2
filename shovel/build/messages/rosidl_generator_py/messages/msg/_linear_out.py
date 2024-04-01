# generated from rosidl_generator_py/resource/_idl.py.em
# with input from messages:msg/LinearOut.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LinearOut(type):
    """Metaclass of message 'LinearOut'."""

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
            module = import_type_support('messages')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'messages.msg.LinearOut')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__linear_out
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__linear_out
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__linear_out
            cls._TYPE_SUPPORT = module.type_support_msg__msg__linear_out
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__linear_out

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LinearOut(metaclass=Metaclass_LinearOut):
    """Message class 'LinearOut'."""

    __slots__ = [
        '_speed',
        '_potentiometer',
        '_time_without_change',
        '_max',
        '_min',
        '_error',
        '_run',
        '_at_min',
        '_at_max',
    ]

    _fields_and_field_types = {
        'speed': 'float',
        'potentiometer': 'int32',
        'time_without_change': 'int32',
        'max': 'int32',
        'min': 'int32',
        'error': 'string',
        'run': 'boolean',
        'at_min': 'boolean',
        'at_max': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.speed = kwargs.get('speed', float())
        self.potentiometer = kwargs.get('potentiometer', int())
        self.time_without_change = kwargs.get('time_without_change', int())
        self.max = kwargs.get('max', int())
        self.min = kwargs.get('min', int())
        self.error = kwargs.get('error', str())
        self.run = kwargs.get('run', bool())
        self.at_min = kwargs.get('at_min', bool())
        self.at_max = kwargs.get('at_max', bool())

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
        if self.speed != other.speed:
            return False
        if self.potentiometer != other.potentiometer:
            return False
        if self.time_without_change != other.time_without_change:
            return False
        if self.max != other.max:
            return False
        if self.min != other.min:
            return False
        if self.error != other.error:
            return False
        if self.run != other.run:
            return False
        if self.at_min != other.at_min:
            return False
        if self.at_max != other.at_max:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def speed(self):
        """Message field 'speed'."""
        return self._speed

    @speed.setter
    def speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed' field must be of type 'float'"
        self._speed = value

    @property
    def potentiometer(self):
        """Message field 'potentiometer'."""
        return self._potentiometer

    @potentiometer.setter
    def potentiometer(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'potentiometer' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'potentiometer' field must be an integer in [-2147483648, 2147483647]"
        self._potentiometer = value

    @property
    def time_without_change(self):
        """Message field 'time_without_change'."""
        return self._time_without_change

    @time_without_change.setter
    def time_without_change(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_without_change' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'time_without_change' field must be an integer in [-2147483648, 2147483647]"
        self._time_without_change = value

    @property  # noqa: A003
    def max(self):  # noqa: A003
        """Message field 'max'."""
        return self._max

    @max.setter  # noqa: A003
    def max(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'max' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'max' field must be an integer in [-2147483648, 2147483647]"
        self._max = value

    @property  # noqa: A003
    def min(self):  # noqa: A003
        """Message field 'min'."""
        return self._min

    @min.setter  # noqa: A003
    def min(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'min' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'min' field must be an integer in [-2147483648, 2147483647]"
        self._min = value

    @property
    def error(self):
        """Message field 'error'."""
        return self._error

    @error.setter
    def error(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'error' field must be of type 'str'"
        self._error = value

    @property
    def run(self):
        """Message field 'run'."""
        return self._run

    @run.setter
    def run(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'run' field must be of type 'bool'"
        self._run = value

    @property
    def at_min(self):
        """Message field 'at_min'."""
        return self._at_min

    @at_min.setter
    def at_min(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'at_min' field must be of type 'bool'"
        self._at_min = value

    @property
    def at_max(self):
        """Message field 'at_max'."""
        return self._at_max

    @at_max.setter
    def at_max(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'at_max' field must be of type 'bool'"
        self._at_max = value
