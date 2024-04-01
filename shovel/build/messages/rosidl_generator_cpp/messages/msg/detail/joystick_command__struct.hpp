// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from messages:msg/JoystickCommand.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__JOYSTICK_COMMAND__STRUCT_HPP_
#define MESSAGES__MSG__DETAIL__JOYSTICK_COMMAND__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__messages__msg__JoystickCommand __attribute__((deprecated))
#else
# define DEPRECATED__messages__msg__JoystickCommand __declspec(deprecated)
#endif

namespace messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JoystickCommand_
{
  using Type = JoystickCommand_<ContainerAllocator>;

  explicit JoystickCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->number = 0;
      this->element = 0;
      this->value = 0.0f;
    }
  }

  explicit JoystickCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->number = 0;
      this->element = 0;
      this->value = 0.0f;
    }
  }

  // field types and members
  using _number_type =
    int8_t;
  _number_type number;
  using _element_type =
    int8_t;
  _element_type element;
  using _value_type =
    float;
  _value_type value;

  // setters for named parameter idiom
  Type & set__number(
    const int8_t & _arg)
  {
    this->number = _arg;
    return *this;
  }
  Type & set__element(
    const int8_t & _arg)
  {
    this->element = _arg;
    return *this;
  }
  Type & set__value(
    const float & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages::msg::JoystickCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages::msg::JoystickCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages::msg::JoystickCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages::msg::JoystickCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages::msg::JoystickCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages::msg::JoystickCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages::msg::JoystickCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages::msg::JoystickCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages::msg::JoystickCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages::msg::JoystickCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages__msg__JoystickCommand
    std::shared_ptr<messages::msg::JoystickCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages__msg__JoystickCommand
    std::shared_ptr<messages::msg::JoystickCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JoystickCommand_ & other) const
  {
    if (this->number != other.number) {
      return false;
    }
    if (this->element != other.element) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const JoystickCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JoystickCommand_

// alias to use template instance with default allocator
using JoystickCommand =
  messages::msg::JoystickCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__JOYSTICK_COMMAND__STRUCT_HPP_
