# generated from rosidl_generator_py/resource/_idl.py.em
# with input from messages:msg/ZedPosition.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ZedPosition(type):
    """Metaclass of message 'ZedPosition'."""

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
                'messages.msg.ZedPosition')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__zed_position
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__zed_position
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__zed_position
            cls._TYPE_SUPPORT = module.type_support_msg__msg__zed_position
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__zed_position

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ZedPosition(metaclass=Metaclass_ZedPosition):
    """Message class 'ZedPosition'."""

    __slots__ = [
        '_x',
        '_y',
        '_z',
        '_ox',
        '_oy',
        '_oz',
        '_ow',
        '_aruco_visible',
        '_pitch',
        '_yaw',
        '_roll',
        '_aruco_pitch',
        '_aruco_yaw',
        '_aruco_roll',
        '_distance',
    ]

    _fields_and_field_types = {
        'x': 'float',
        'y': 'float',
        'z': 'float',
        'ox': 'float',
        'oy': 'float',
        'oz': 'float',
        'ow': 'float',
        'aruco_visible': 'boolean',
        'pitch': 'float',
        'yaw': 'float',
        'roll': 'float',
        'aruco_pitch': 'float',
        'aruco_yaw': 'float',
        'aruco_roll': 'float',
        'distance': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
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
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.z = kwargs.get('z', float())
        self.ox = kwargs.get('ox', float())
        self.oy = kwargs.get('oy', float())
        self.oz = kwargs.get('oz', float())
        self.ow = kwargs.get('ow', float())
        self.aruco_visible = kwargs.get('aruco_visible', bool())
        self.pitch = kwargs.get('pitch', float())
        self.yaw = kwargs.get('yaw', float())
        self.roll = kwargs.get('roll', float())
        self.aruco_pitch = kwargs.get('aruco_pitch', float())
        self.aruco_yaw = kwargs.get('aruco_yaw', float())
        self.aruco_roll = kwargs.get('aruco_roll', float())
        self.distance = kwargs.get('distance', float())

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
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        if self.ox != other.ox:
            return False
        if self.oy != other.oy:
            return False
        if self.oz != other.oz:
            return False
        if self.ow != other.ow:
            return False
        if self.aruco_visible != other.aruco_visible:
            return False
        if self.pitch != other.pitch:
            return False
        if self.yaw != other.yaw:
            return False
        if self.roll != other.roll:
            return False
        if self.aruco_pitch != other.aruco_pitch:
            return False
        if self.aruco_yaw != other.aruco_yaw:
            return False
        if self.aruco_roll != other.aruco_roll:
            return False
        if self.distance != other.distance:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
        self._x = value

    @property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
        self._y = value

    @property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z' field must be of type 'float'"
        self._z = value

    @property
    def ox(self):
        """Message field 'ox'."""
        return self._ox

    @ox.setter
    def ox(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ox' field must be of type 'float'"
        self._ox = value

    @property
    def oy(self):
        """Message field 'oy'."""
        return self._oy

    @oy.setter
    def oy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'oy' field must be of type 'float'"
        self._oy = value

    @property
    def oz(self):
        """Message field 'oz'."""
        return self._oz

    @oz.setter
    def oz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'oz' field must be of type 'float'"
        self._oz = value

    @property
    def ow(self):
        """Message field 'ow'."""
        return self._ow

    @ow.setter
    def ow(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ow' field must be of type 'float'"
        self._ow = value

    @property
    def aruco_visible(self):
        """Message field 'aruco_visible'."""
        return self._aruco_visible

    @aruco_visible.setter
    def aruco_visible(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'aruco_visible' field must be of type 'bool'"
        self._aruco_visible = value

    @property
    def pitch(self):
        """Message field 'pitch'."""
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch' field must be of type 'float'"
        self._pitch = value

    @property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
        self._yaw = value

    @property
    def roll(self):
        """Message field 'roll'."""
        return self._roll

    @roll.setter
    def roll(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll' field must be of type 'float'"
        self._roll = value

    @property
    def aruco_pitch(self):
        """Message field 'aruco_pitch'."""
        return self._aruco_pitch

    @aruco_pitch.setter
    def aruco_pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aruco_pitch' field must be of type 'float'"
        self._aruco_pitch = value

    @property
    def aruco_yaw(self):
        """Message field 'aruco_yaw'."""
        return self._aruco_yaw

    @aruco_yaw.setter
    def aruco_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aruco_yaw' field must be of type 'float'"
        self._aruco_yaw = value

    @property
    def aruco_roll(self):
        """Message field 'aruco_roll'."""
        return self._aruco_roll

    @aruco_roll.setter
    def aruco_roll(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aruco_roll' field must be of type 'float'"
        self._aruco_roll = value

    @property
    def distance(self):
        """Message field 'distance'."""
        return self._distance

    @distance.setter
    def distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance' field must be of type 'float'"
        self._distance = value
