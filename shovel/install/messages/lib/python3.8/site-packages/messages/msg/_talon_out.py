# generated from rosidl_generator_py/resource/_idl.py.em
# with input from messages:msg/TalonOut.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TalonOut(type):
    """Metaclass of message 'TalonOut'."""

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
                'messages.msg.TalonOut')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__talon_out
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__talon_out
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__talon_out
            cls._TYPE_SUPPORT = module.type_support_msg__msg__talon_out
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__talon_out

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TalonOut(metaclass=Metaclass_TalonOut):
    """Message class 'TalonOut'."""

    __slots__ = [
        '_device_id',
        '_bus_voltage',
        '_output_current',
        '_output_voltage',
        '_output_percent',
        '_temperature',
        '_sensor_position',
        '_sensor_velocity',
        '_closed_loop_error',
        '_integral_accumulator',
        '_error_derivative',
        '_potentiometer',
    ]

    _fields_and_field_types = {
        'device_id': 'int32',
        'bus_voltage': 'float',
        'output_current': 'float',
        'output_voltage': 'float',
        'output_percent': 'float',
        'temperature': 'float',
        'sensor_position': 'int32',
        'sensor_velocity': 'int32',
        'closed_loop_error': 'int32',
        'integral_accumulator': 'int32',
        'error_derivative': 'int32',
        'potentiometer': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.device_id = kwargs.get('device_id', int())
        self.bus_voltage = kwargs.get('bus_voltage', float())
        self.output_current = kwargs.get('output_current', float())
        self.output_voltage = kwargs.get('output_voltage', float())
        self.output_percent = kwargs.get('output_percent', float())
        self.temperature = kwargs.get('temperature', float())
        self.sensor_position = kwargs.get('sensor_position', int())
        self.sensor_velocity = kwargs.get('sensor_velocity', int())
        self.closed_loop_error = kwargs.get('closed_loop_error', int())
        self.integral_accumulator = kwargs.get('integral_accumulator', int())
        self.error_derivative = kwargs.get('error_derivative', int())
        self.potentiometer = kwargs.get('potentiometer', int())

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
        if self.device_id != other.device_id:
            return False
        if self.bus_voltage != other.bus_voltage:
            return False
        if self.output_current != other.output_current:
            return False
        if self.output_voltage != other.output_voltage:
            return False
        if self.output_percent != other.output_percent:
            return False
        if self.temperature != other.temperature:
            return False
        if self.sensor_position != other.sensor_position:
            return False
        if self.sensor_velocity != other.sensor_velocity:
            return False
        if self.closed_loop_error != other.closed_loop_error:
            return False
        if self.integral_accumulator != other.integral_accumulator:
            return False
        if self.error_derivative != other.error_derivative:
            return False
        if self.potentiometer != other.potentiometer:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def device_id(self):
        """Message field 'device_id'."""
        return self._device_id

    @device_id.setter
    def device_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'device_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'device_id' field must be an integer in [-2147483648, 2147483647]"
        self._device_id = value

    @property
    def bus_voltage(self):
        """Message field 'bus_voltage'."""
        return self._bus_voltage

    @bus_voltage.setter
    def bus_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bus_voltage' field must be of type 'float'"
        self._bus_voltage = value

    @property
    def output_current(self):
        """Message field 'output_current'."""
        return self._output_current

    @output_current.setter
    def output_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'output_current' field must be of type 'float'"
        self._output_current = value

    @property
    def output_voltage(self):
        """Message field 'output_voltage'."""
        return self._output_voltage

    @output_voltage.setter
    def output_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'output_voltage' field must be of type 'float'"
        self._output_voltage = value

    @property
    def output_percent(self):
        """Message field 'output_percent'."""
        return self._output_percent

    @output_percent.setter
    def output_percent(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'output_percent' field must be of type 'float'"
        self._output_percent = value

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
    def sensor_position(self):
        """Message field 'sensor_position'."""
        return self._sensor_position

    @sensor_position.setter
    def sensor_position(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sensor_position' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'sensor_position' field must be an integer in [-2147483648, 2147483647]"
        self._sensor_position = value

    @property
    def sensor_velocity(self):
        """Message field 'sensor_velocity'."""
        return self._sensor_velocity

    @sensor_velocity.setter
    def sensor_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sensor_velocity' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'sensor_velocity' field must be an integer in [-2147483648, 2147483647]"
        self._sensor_velocity = value

    @property
    def closed_loop_error(self):
        """Message field 'closed_loop_error'."""
        return self._closed_loop_error

    @closed_loop_error.setter
    def closed_loop_error(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'closed_loop_error' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'closed_loop_error' field must be an integer in [-2147483648, 2147483647]"
        self._closed_loop_error = value

    @property
    def integral_accumulator(self):
        """Message field 'integral_accumulator'."""
        return self._integral_accumulator

    @integral_accumulator.setter
    def integral_accumulator(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'integral_accumulator' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'integral_accumulator' field must be an integer in [-2147483648, 2147483647]"
        self._integral_accumulator = value

    @property
    def error_derivative(self):
        """Message field 'error_derivative'."""
        return self._error_derivative

    @error_derivative.setter
    def error_derivative(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_derivative' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'error_derivative' field must be an integer in [-2147483648, 2147483647]"
        self._error_derivative = value

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
