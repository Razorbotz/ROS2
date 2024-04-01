// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from messages:msg/TalonOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__TALON_OUT__STRUCT_HPP_
#define MESSAGES__MSG__DETAIL__TALON_OUT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__messages__msg__TalonOut __attribute__((deprecated))
#else
# define DEPRECATED__messages__msg__TalonOut __declspec(deprecated)
#endif

namespace messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TalonOut_
{
  using Type = TalonOut_<ContainerAllocator>;

  explicit TalonOut_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->device_id = 0l;
      this->bus_voltage = 0.0f;
      this->output_current = 0.0f;
      this->output_voltage = 0.0f;
      this->output_percent = 0.0f;
      this->temperature = 0.0f;
      this->sensor_position = 0l;
      this->sensor_velocity = 0l;
      this->closed_loop_error = 0l;
      this->integral_accumulator = 0l;
      this->error_derivative = 0l;
      this->potentiometer = 0l;
    }
  }

  explicit TalonOut_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->device_id = 0l;
      this->bus_voltage = 0.0f;
      this->output_current = 0.0f;
      this->output_voltage = 0.0f;
      this->output_percent = 0.0f;
      this->temperature = 0.0f;
      this->sensor_position = 0l;
      this->sensor_velocity = 0l;
      this->closed_loop_error = 0l;
      this->integral_accumulator = 0l;
      this->error_derivative = 0l;
      this->potentiometer = 0l;
    }
  }

  // field types and members
  using _device_id_type =
    int32_t;
  _device_id_type device_id;
  using _bus_voltage_type =
    float;
  _bus_voltage_type bus_voltage;
  using _output_current_type =
    float;
  _output_current_type output_current;
  using _output_voltage_type =
    float;
  _output_voltage_type output_voltage;
  using _output_percent_type =
    float;
  _output_percent_type output_percent;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _sensor_position_type =
    int32_t;
  _sensor_position_type sensor_position;
  using _sensor_velocity_type =
    int32_t;
  _sensor_velocity_type sensor_velocity;
  using _closed_loop_error_type =
    int32_t;
  _closed_loop_error_type closed_loop_error;
  using _integral_accumulator_type =
    int32_t;
  _integral_accumulator_type integral_accumulator;
  using _error_derivative_type =
    int32_t;
  _error_derivative_type error_derivative;
  using _potentiometer_type =
    int32_t;
  _potentiometer_type potentiometer;

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
  Type & set__output_current(
    const float & _arg)
  {
    this->output_current = _arg;
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
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__sensor_position(
    const int32_t & _arg)
  {
    this->sensor_position = _arg;
    return *this;
  }
  Type & set__sensor_velocity(
    const int32_t & _arg)
  {
    this->sensor_velocity = _arg;
    return *this;
  }
  Type & set__closed_loop_error(
    const int32_t & _arg)
  {
    this->closed_loop_error = _arg;
    return *this;
  }
  Type & set__integral_accumulator(
    const int32_t & _arg)
  {
    this->integral_accumulator = _arg;
    return *this;
  }
  Type & set__error_derivative(
    const int32_t & _arg)
  {
    this->error_derivative = _arg;
    return *this;
  }
  Type & set__potentiometer(
    const int32_t & _arg)
  {
    this->potentiometer = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages::msg::TalonOut_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages::msg::TalonOut_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages::msg::TalonOut_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages::msg::TalonOut_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages::msg::TalonOut_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages::msg::TalonOut_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages::msg::TalonOut_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages::msg::TalonOut_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages::msg::TalonOut_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages::msg::TalonOut_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages__msg__TalonOut
    std::shared_ptr<messages::msg::TalonOut_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages__msg__TalonOut
    std::shared_ptr<messages::msg::TalonOut_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TalonOut_ & other) const
  {
    if (this->device_id != other.device_id) {
      return false;
    }
    if (this->bus_voltage != other.bus_voltage) {
      return false;
    }
    if (this->output_current != other.output_current) {
      return false;
    }
    if (this->output_voltage != other.output_voltage) {
      return false;
    }
    if (this->output_percent != other.output_percent) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->sensor_position != other.sensor_position) {
      return false;
    }
    if (this->sensor_velocity != other.sensor_velocity) {
      return false;
    }
    if (this->closed_loop_error != other.closed_loop_error) {
      return false;
    }
    if (this->integral_accumulator != other.integral_accumulator) {
      return false;
    }
    if (this->error_derivative != other.error_derivative) {
      return false;
    }
    if (this->potentiometer != other.potentiometer) {
      return false;
    }
    return true;
  }
  bool operator!=(const TalonOut_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TalonOut_

// alias to use template instance with default allocator
using TalonOut =
  messages::msg::TalonOut_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__TALON_OUT__STRUCT_HPP_
