// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from messages:msg/KeyState.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__KEY_STATE__STRUCT_HPP_
#define MESSAGES__MSG__DETAIL__KEY_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__messages__msg__KeyState __attribute__((deprecated))
#else
# define DEPRECATED__messages__msg__KeyState __declspec(deprecated)
#endif

namespace messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct KeyState_
{
  using Type = KeyState_<ContainerAllocator>;

  explicit KeyState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->key = 0;
      this->state = 0;
    }
  }

  explicit KeyState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->key = 0;
      this->state = 0;
    }
  }

  // field types and members
  using _key_type =
    uint16_t;
  _key_type key;
  using _state_type =
    uint8_t;
  _state_type state;

  // setters for named parameter idiom
  Type & set__key(
    const uint16_t & _arg)
  {
    this->key = _arg;
    return *this;
  }
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages::msg::KeyState_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages::msg::KeyState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages::msg::KeyState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages::msg::KeyState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages::msg::KeyState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages::msg::KeyState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages::msg::KeyState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages::msg::KeyState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages::msg::KeyState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages::msg::KeyState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages__msg__KeyState
    std::shared_ptr<messages::msg::KeyState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages__msg__KeyState
    std::shared_ptr<messages::msg::KeyState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const KeyState_ & other) const
  {
    if (this->key != other.key) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const KeyState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct KeyState_

// alias to use template instance with default allocator
using KeyState =
  messages::msg::KeyState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__KEY_STATE__STRUCT_HPP_
