// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from messages:msg/VictorOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__VICTOR_OUT__STRUCT_HPP_
#define MESSAGES__MSG__DETAIL__VICTOR_OUT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__messages__msg__VictorOut __attribute__((deprecated))
#else
# define DEPRECATED__messages__msg__VictorOut __declspec(deprecated)
#endif

namespace messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VictorOut_
{
  using Type = VictorOut_<ContainerAllocator>;

  explicit VictorOut_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->device_id = 0l;
      this->bus_voltage = 0.0f;
      this->output_voltage = 0.0f;
      this->output_percent = 0.0f;
    }
  }

  explicit VictorOut_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->device_id = 0l;
      this->bus_voltage = 0.0f;
      this->output_voltage = 0.0f;
      this->output_percent = 0.0f;
    }
  }

  // field types and members
  using _device_id_type =
    int32_t;
  _device_id_type device_id;
  using _bus_voltage_type =
    float;
  _bus_voltage_type bus_voltage;
  using _output_voltage_type =
    float;
  _output_voltage_type output_voltage;
  using _output_percent_type =
    float;
  _output_percent_type output_percent;

  // setters for named parameter idiom
  Type & set__device_id(
    const int32_t & _arg)
  {
    this->device_id = _arg;
    return *this;
  }
  Type & set__bus_voltage(
    const float & _arg)
  {
    this->bus_voltage = _arg;
    return *this;
  }
  Type & set__output_voltage(
    const float & _arg)
  {
    this->output_voltage = _arg;
    return *this;
  }
  Type & set__output_percent(
    const float & _arg)
  {
    this->output_percent = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages::msg::VictorOut_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages::msg::VictorOut_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages::msg::VictorOut_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages::msg::VictorOut_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages::msg::VictorOut_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages::msg::VictorOut_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages::msg::VictorOut_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages::msg::VictorOut_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages::msg::VictorOut_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages::msg::VictorOut_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages__msg__VictorOut
    std::shared_ptr<messages::msg::VictorOut_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages__msg__VictorOut
    std::shared_ptr<messages::msg::VictorOut_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VictorOut_ & other) const
  {
    if (this->device_id != other.device_id) {
      return false;
    }
    if (this->bus_voltage != other.bus_voltage) {
      return false;
    }
    if (this->output_voltage != other.output_voltage) {
      return false;
    }
    if (this->output_percent != other.output_percent) {
      return false;
    }
    return true;
  }
  bool operator!=(const VictorOut_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VictorOut_

// alias to use template instance with default allocator
using VictorOut =
  messages::msg::VictorOut_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__VICTOR_OUT__STRUCT_HPP_
