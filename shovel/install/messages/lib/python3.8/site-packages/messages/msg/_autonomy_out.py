# generated from rosidl_generator_py/resource/_idl.py.em
# with input from messages:msg/AutonomyOut.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AutonomyOut(type):
    """Metaclass of message 'AutonomyOut'."""

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
                'messages.msg.AutonomyOut')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__autonomy_out
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__autonomy_out
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__autonomy_out
            cls._TYPE_SUPPORT = module.type_support_msg__msg__autonomy_out
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__autonomy_out

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AutonomyOut(metaclass=Metaclass_AutonomyOut):
    """Message class 'AutonomyOut'."""

    __slots__ = [
        '_robot_state',
        '_excavation_state',
        '_error_state',
        '_dump_state',
    ]

    _fields_and_field_types = {
        'robot_state': 'string',
        'excavation_state': 'string',
        'error_state': 'string',
        'dump_state': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robot_state = kwargs.get('robot_state', str())
        self.excavation_state = kwargs.get('excavation_state', str())
        self.error_state = kwargs.get('error_state', str())
        self.dump_state = kwargs.get('dump_state', str())

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
        if self.robot_state != other.robot_state:
            return False
        if self.excavation_state != other.excavation_state:
            return False
        if self.error_state != other.error_state:
            return False
        if self.dump_state != other.dump_state:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def robot_state(self):
        """Message field 'robot_state'."""
        return self._robot_state

    @robot_state.setter
    def robot_state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'robot_state' field must be of type 'str'"
        self._robot_state = value

    @property
    def excavation_state(self):
        """Message field 'excavation_state'."""
        return self._excavation_state

    @excavation_state.setter
    def excavation_state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'excavation_state' field must be of type 'str'"
        self._excavation_state = value

    @property
    def error_state(self):
        """Message field 'error_state'."""
        return self._error_state

    @error_state.setter
    def error_state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'error_state' field must be of type 'str'"
        self._error_state = value

    @property
    def dump_state(self):
        """Message field 'dump_state'."""
        return self._dump_state

    @dump_state.setter
    def dump_state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'dump_state' field must be of type 'str'"
        self._dump_state = value
