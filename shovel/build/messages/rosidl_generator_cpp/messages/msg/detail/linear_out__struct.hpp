// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from messages:msg/LinearOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__LINEAR_OUT__STRUCT_HPP_
#define MESSAGES__MSG__DETAIL__LINEAR_OUT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__messages__msg__LinearOut __attribute__((deprecated))
#else
# define DEPRECATED__messages__msg__LinearOut __declspec(deprecated)
#endif

namespace messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LinearOut_
{
  using Type = LinearOut_<ContainerAllocator>;

  explicit LinearOut_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0f;
      this->potentiometer = 0l;
      this->time_without_change = 0l;
      this->max = 0l;
      this->min = 0l;
      this->error = "";
      this->run = false;
      this->at_min = false;
      this->at_max = false;
    }
  }

  explicit LinearOut_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0f;
      this->potentiometer = 0l;
      this->time_without_change = 0l;
      this->max = 0l;
      this->min = 0l;
      this->error = "";
      this->run = false;
      this->at_min = false;
      this->at_max = false;
    }
  }

  // field types and members
  using _speed_type =
    float;
  _speed_type speed;
  using _potentiometer_type =
    int32_t;
  _potentiometer_type potentiometer;
  using _time_without_change_type =
    int32_t;
  _time_without_change_type time_without_change;
  using _max_type =
    int32_t;
  _max_type max;
  using _min_type =
    int32_t;
  _min_type min;
  using _error_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_type error;
  using _run_type =
    bool;
  _run_type run;
  using _at_min_type =
    bool;
  _at_min_type at_min;
  using _at_max_type =
    bool;
  _at_max_type at_max;

  // setters for named parameter idiom
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__potentiometer(
    const int32_t & _arg)
  {
    this->potentiometer = _arg;
    return *this;
  }
  Type & set__time_without_change(
    const int32_t & _arg)
  {
    this->time_without_change = _arg;
    return *this;
  }
  Type & set__max(
    const int32_t & _arg)
  {
    this->max = _arg;
    return *this;
  }
  Type & set__min(
    const int32_t & _arg)
  {
    this->min = _arg;
    return *this;
  }
  Type & set__error(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__run(
    const bool & _arg)
  {
    this->run = _arg;
    return *this;
  }
  Type & set__at_min(
    const bool & _arg)
  {
    this->at_min = _arg;
    return *this;
  }
  Type & set__at_max(
    const bool & _arg)
  {
    this->at_max = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages::msg::LinearOut_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages::msg::LinearOut_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages::msg::LinearOut_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages::msg::LinearOut_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages::msg::LinearOut_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages::msg::LinearOut_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages::msg::LinearOut_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages::msg::LinearOut_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages::msg::LinearOut_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages::msg::LinearOut_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages__msg__LinearOut
    std::shared_ptr<messages::msg::LinearOut_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages__msg__LinearOut
    std::shared_ptr<messages::msg::LinearOut_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LinearOut_ & other) const
  {
    if (this->speed != other.speed) {
      return false;
    }
    if (this->potentiometer != other.potentiometer) {
      return false;
    }
    if (this->time_without_change != other.time_without_change) {
      return false;
    }
    if (this->max != other.max) {
      return false;
    }
    if (this->min != other.min) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    if (this->run != other.run) {
      return false;
    }
    if (this->at_min != other.at_min) {
      return false;
    }
    if (this->at_max != other.at_max) {
      return false;
    }
    return true;
  }
  bool operator!=(const LinearOut_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LinearOut_

// alias to use template instance with default allocator
using LinearOut =
  messages::msg::LinearOut_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__LINEAR_OUT__STRUCT_HPP_
