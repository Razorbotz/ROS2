# generated from rosidl_generator_py/resource/_idl.py.em
# with input from messages:msg/Power.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Power(type):
    """Metaclass of message 'Power'."""

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
                'messages.msg.Power')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__power
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__power
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__power
            cls._TYPE_SUPPORT = module.type_support_msg__msg__power
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__power

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Power(metaclass=Metaclass_Power):
    """Message class 'Power'."""

    __slots__ = [
        '_voltage',
        '_temperature',
        '_current0',
        '_current1',
        '_current2',
        '_current3',
        '_current4',
        '_current5',
        '_current6',
        '_current7',
        '_current8',
        '_current9',
        '_current10',
        '_current11',
        '_current12',
        '_current13',
        '_current14',
        '_current15',
    ]

    _fields_and_field_types = {
        'voltage': 'float',
        'temperature': 'float',
        'current0': 'float',
        'current1': 'float',
        'current2': 'float',
        'current3': 'float',
        'current4': 'float',
        'current5': 'float',
        'current6': 'float',
        'current7': 'float',
        'current8': 'float',
        'current9': 'float',
        'current10': 'float',
        'current11': 'float',
        'current12': 'float',
        'current13': 'float',
        'current14': 'float',
        'current15': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.voltage = kwargs.get('voltage', float())
        self.temperature = kwargs.get('temperature', float())
        self.current0 = kwargs.get('current0', float())
        self.current1 = kwargs.get('current1', float())
        self.current2 = kwargs.get('current2', float())
        self.current3 = kwargs.get('current3', float())
        self.current4 = kwargs.get('current4', float())
        self.current5 = kwargs.get('current5', float())
        self.current6 = kwargs.get('current6', float())
        self.current7 = kwargs.get('current7', float())
        self.current8 = kwargs.get('current8', float())
        self.current9 = kwargs.get('current9', float())
        self.current10 = kwargs.get('current10', float())
        self.current11 = kwargs.get('current11', float())
        self.current12 = kwargs.get('current12', float())
        self.current13 = kwargs.get('current13', float())
        self.current14 = kwargs.get('current14', float())
        self.current15 = kwargs.get('current15', float())

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
        if self.voltage != other.voltage:
            return False
        if self.temperature != other.temperature:
            return False
        if self.current0 != other.current0:
            return False
        if self.current1 != other.current1:
            return False
        if self.current2 != other.current2:
            return False
        if self.current3 != other.current3:
            return False
        if self.current4 != other.current4:
            return False
        if self.current5 != other.current5:
            return False
        if self.current6 != other.current6:
            return False
        if self.current7 != other.current7:
            return False
        if self.current8 != other.current8:
            return False
        if self.current9 != other.current9:
            return False
        if self.current10 != other.current10:
            return False
        if self.current11 != other.current11:
            return False
        if self.current12 != other.current12:
            return False
        if self.current13 != other.current13:
            return False
        if self.current14 != other.current14:
            return False
        if self.current15 != other.current15:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def voltage(self):
        """Message field 'voltage'."""
        return self._voltage

    @voltage.setter
    def voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'voltage' field must be of type 'float'"
        self._voltage = value

    @property
    def temperature(self):
        """Message field 'temperature'."""
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'temperature' field must be of type 'float'"
        self._temperature = value

    @property
    def current0(self):
        """Message field 'current0'."""
        return self._current0

    @current0.setter
    def current0(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current0' field must be of type 'float'"
        self._current0 = value

    @property
    def current1(self):
        """Message field 'current1'."""
        return self._current1

    @current1.setter
    def current1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current1' field must be of type 'float'"
        self._current1 = value

    @property
    def current2(self):
        """Message field 'current2'."""
        return self._current2

    @current2.setter
    def current2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current2' field must be of type 'float'"
        self._current2 = value

    @property
    def current3(self):
        """Message field 'current3'."""
        return self._current3

    @current3.setter
    def current3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current3' field must be of type 'float'"
        self._current3 = value

    @property
    def current4(self):
        """Message field 'current4'."""
        return self._current4

    @current4.setter
    def current4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current4' field must be of type 'float'"
        self._current4 = value

    @property
    def current5(self):
        """Message field 'current5'."""
        return self._current5

    @current5.setter
    def current5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current5' field must be of type 'float'"
        self._current5 = value

    @property
    def current6(self):
        """Message field 'current6'."""
        return self._current6

    @current6.setter
    def current6(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current6' field must be of type 'float'"
        self._current6 = value

    @property
    def current7(self):
        """Message field 'current7'."""
        return self._current7

    @current7.setter
    def current7(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current7' field must be of type 'float'"
        self._current7 = value

    @property
    def current8(self):
        """Message field 'current8'."""
        return self._current8

    @current8.setter
    def current8(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current8' field must be of type 'float'"
        self._current8 = value

    @property
    def current9(self):
        """Message field 'current9'."""
        return self._current9

    @current9.setter
    def current9(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current9' field must be of type 'float'"
        self._current9 = value

    @property
    def current10(self):
        """Message field 'current10'."""
        return self._current10

    @current10.setter
    def current10(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current10' field must be of type 'float'"
        self._current10 = value

    @property
    def current11(self):
        """Message field 'current11'."""
        return self._current11

    @current11.setter
    def current11(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current11' field must be of type 'float'"
        self._current11 = value

    @property
    def current12(self):
        """Message field 'current12'."""
        return self._current12

    @current12.setter
    def current12(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current12' field must be of type 'float'"
        self._current12 = value

    @property
    def current13(self):
        """Message field 'current13'."""
        return self._current13

    @current13.setter
    def current13(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current13' field must be of type 'float'"
        self._current13 = value

    @property
    def current14(self):
        """Message field 'current14'."""
        return self._current14

    @current14.setter
    def current14(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current14' field must be of type 'float'"
        self._current14 = value

    @property
    def current15(self):
        """Message field 'current15'."""
        return self._current15

    @current15.setter
    def current15(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current15' field must be of type 'float'"
        self._current15 = value
