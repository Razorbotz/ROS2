// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from messages:msg/Power.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__POWER__STRUCT_HPP_
#define MESSAGES__MSG__DETAIL__POWER__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__messages__msg__Power __attribute__((deprecated))
#else
# define DEPRECATED__messages__msg__Power __declspec(deprecated)
#endif

namespace messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Power_
{
  using Type = Power_<ContainerAllocator>;

  explicit Power_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->voltage = 0.0f;
      this->temperature = 0.0f;
      this->current0 = 0.0f;
      this->current1 = 0.0f;
      this->current2 = 0.0f;
      this->current3 = 0.0f;
      this->current4 = 0.0f;
      this->current5 = 0.0f;
      this->current6 = 0.0f;
      this->current7 = 0.0f;
      this->current8 = 0.0f;
      this->current9 = 0.0f;
      this->current10 = 0.0f;
      this->current11 = 0.0f;
      this->current12 = 0.0f;
      this->current13 = 0.0f;
      this->current14 = 0.0f;
      this->current15 = 0.0f;
    }
  }

  explicit Power_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->voltage = 0.0f;
      this->temperature = 0.0f;
      this->current0 = 0.0f;
      this->current1 = 0.0f;
      this->current2 = 0.0f;
      this->current3 = 0.0f;
      this->current4 = 0.0f;
      this->current5 = 0.0f;
      this->current6 = 0.0f;
      this->current7 = 0.0f;
      this->current8 = 0.0f;
      this->current9 = 0.0f;
      this->current10 = 0.0f;
      this->current11 = 0.0f;
      this->current12 = 0.0f;
      this->current13 = 0.0f;
      this->current14 = 0.0f;
      this->current15 = 0.0f;
    }
  }

  // field types and members
  using _voltage_type =
    float;
  _voltage_type voltage;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _current0_type =
    float;
  _current0_type current0;
  using _current1_type =
    float;
  _current1_type current1;
  using _current2_type =
    float;
  _current2_type current2;
  using _current3_type =
    float;
  _current3_type current3;
  using _current4_type =
    float;
  _current4_type current4;
  using _current5_type =
    float;
  _current5_type current5;
  using _current6_type =
    float;
  _current6_type current6;
  using _current7_type =
    float;
  _current7_type current7;
  using _current8_type =
    float;
  _current8_type current8;
  using _current9_type =
    float;
  _current9_type current9;
  using _current10_type =
    float;
  _current10_type current10;
  using _current11_type =
    float;
  _current11_type current11;
  using _current12_type =
    float;
  _current12_type current12;
  using _current13_type =
    float;
  _current13_type current13;
  using _current14_type =
    float;
  _current14_type current14;
  using _current15_type =
    float;
  _current15_type current15;

  // setters for named parameter idiom
  Type & set__voltage(
    const float & _arg)
  {
    this->voltage = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__current0(
    const float & _arg)
  {
    this->current0 = _arg;
    return *this;
  }
  Type & set__current1(
    const float & _arg)
  {
    this->current1 = _arg;
    return *this;
  }
  Type & set__current2(
    const float & _arg)
  {
    this->current2 = _arg;
    return *this;
  }
  Type & set__current3(
    const float & _arg)
  {
    this->current3 = _arg;
    return *this;
  }
  Type & set__current4(
    const float & _arg)
  {
    this->current4 = _arg;
    return *this;
  }
  Type & set__current5(
    const float & _arg)
  {
    this->current5 = _arg;
    return *this;
  }
  Type & set__current6(
    const float & _arg)
  {
    this->current6 = _arg;
    return *this;
  }
  Type & set__current7(
    const float & _arg)
  {
    this->current7 = _arg;
    return *this;
  }
  Type & set__current8(
    const float & _arg)
  {
    this->current8 = _arg;
    return *this;
  }
  Type & set__current9(
    const float & _arg)
  {
    this->current9 = _arg;
    return *this;
  }
  Type & set__current10(
    const float & _arg)
  {
    this->current10 = _arg;
    return *this;
  }
  Type & set__current11(
    const float & _arg)
  {
    this->current11 = _arg;
    return *this;
  }
  Type & set__current12(
    const float & _arg)
  {
    this->current12 = _arg;
    return *this;
  }
  Type & set__current13(
    const float & _arg)
  {
    this->current13 = _arg;
    return *this;
  }
  Type & set__current14(
    const float & _arg)
  {
    this->current14 = _arg;
    return *this;
  }
  Type & set__current15(
    const float & _arg)
  {
    this->current15 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages::msg::Power_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages::msg::Power_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages::msg::Power_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages::msg::Power_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages::msg::Power_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages::msg::Power_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages::msg::Power_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages::msg::Power_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages::msg::Power_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages::msg::Power_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages__msg__Power
    std::shared_ptr<messages::msg::Power_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages__msg__Power
    std::shared_ptr<messages::msg::Power_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Power_ & other) const
  {
    if (this->voltage != other.voltage) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->current0 != other.current0) {
      return false;
    }
    if (this->current1 != other.current1) {
      return false;
    }
    if (this->current2 != other.current2) {
      return false;
    }
    if (this->current3 != other.current3) {
      return false;
    }
    if (this->current4 != other.current4) {
      return false;
    }
    if (this->current5 != other.current5) {
      return false;
    }
    if (this->current6 != other.current6) {
      return false;
    }
    if (this->current7 != other.current7) {
      return false;
    }
    if (this->current8 != other.current8) {
      return false;
    }
    if (this->current9 != other.current9) {
      return false;
    }
    if (this->current10 != other.current10) {
      return false;
    }
    if (this->current11 != other.current11) {
      return false;
    }
    if (this->current12 != other.current12) {
      return false;
    }
    if (this->current13 != other.current13) {
      return false;
    }
    if (this->current14 != other.current14) {
      return false;
    }
    if (this->current15 != other.current15) {
      return false;
    }
    return true;
  }
  bool operator!=(const Power_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Power_

// alias to use template instance with default allocator
using Power =
  messages::msg::Power_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__POWER__STRUCT_HPP_
