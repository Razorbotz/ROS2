// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from messages:msg/AutonomyOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__AUTONOMY_OUT__STRUCT_HPP_
#define MESSAGES__MSG__DETAIL__AUTONOMY_OUT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__messages__msg__AutonomyOut __attribute__((deprecated))
#else
# define DEPRECATED__messages__msg__AutonomyOut __declspec(deprecated)
#endif

namespace messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AutonomyOut_
{
  using Type = AutonomyOut_<ContainerAllocator>;

  explicit AutonomyOut_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_state = "";
      this->excavation_state = "";
      this->error_state = "";
      this->dump_state = "";
    }
  }

  explicit AutonomyOut_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_state(_alloc),
    excavation_state(_alloc),
    error_state(_alloc),
    dump_state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_state = "";
      this->excavation_state = "";
      this->error_state = "";
      this->dump_state = "";
    }
  }

  // field types and members
  using _robot_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_state_type robot_state;
  using _excavation_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _excavation_state_type excavation_state;
  using _error_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_state_type error_state;
  using _dump_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _dump_state_type dump_state;

  // setters for named parameter idiom
  Type & set__robot_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_state = _arg;
    return *this;
  }
  Type & set__excavation_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->excavation_state = _arg;
    return *this;
  }
  Type & set__error_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_state = _arg;
    return *this;
  }
  Type & set__dump_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->dump_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages::msg::AutonomyOut_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages::msg::AutonomyOut_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages::msg::AutonomyOut_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages::msg::AutonomyOut_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages::msg::AutonomyOut_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages::msg::AutonomyOut_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages::msg::AutonomyOut_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages::msg::AutonomyOut_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages::msg::AutonomyOut_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages::msg::AutonomyOut_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages__msg__AutonomyOut
    std::shared_ptr<messages::msg::AutonomyOut_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages__msg__AutonomyOut
    std::shared_ptr<messages::msg::AutonomyOut_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AutonomyOut_ & other) const
  {
    if (this->robot_state != other.robot_state) {
      return false;
    }
    if (this->excavation_state != other.excavation_state) {
      return false;
    }
    if (this->error_state != other.error_state) {
      return false;
    }
    if (this->dump_state != other.dump_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const AutonomyOut_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AutonomyOut_

// alias to use template instance with default allocator
using AutonomyOut =
  messages::msg::AutonomyOut_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__AUTONOMY_OUT__STRUCT_HPP_
