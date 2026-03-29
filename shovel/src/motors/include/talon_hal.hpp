#pragma once

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Hardware abstraction layer for Talon SRX motor controllers.
 *
 * Provides a uniform interface for both physical hardware (Phoenix 5)
 * and Gazebo simulation. The rest of the node code interacts only
 * with this interface, eliminating behavior differences between
 * the physical and simulation stacks.
 */
class TalonHAL {
public:
    virtual ~TalonHAL() = default;

    // --- Lifecycle ---
    virtual bool initialize(rclcpp::Node::SharedPtr node, int motorNumber,
                            const std::string& canInterface, int diagnosticsPort,
                            bool invertMotor, double kP, double kI,
                            double kD, double kF) = 0;

    // --- Control ---
    virtual void setPercentOutput(double percent) = 0;
    virtual void setPosition(int position) = 0;
    virtual void feedEnable(int timeoutMs) = 0;

    // --- Feedback ---
    virtual int    getDeviceID() = 0;
    virtual double getBusVoltage() = 0;
    virtual double getOutputCurrent() = 0;
    virtual double getMotorOutputVoltage() = 0;
    virtual double getMotorOutputPercent() = 0;
    virtual double getTemperature() = 0;
    virtual int    getSensorPosition() = 0;
    virtual double getSensorVelocity() = 0;
    virtual int    getClosedLoopError() = 0;
    virtual double getIntegralAccumulator() = 0;
    virtual double getErrorDerivative() = 0;

    // --- Factory ---
    static std::unique_ptr<TalonHAL> create(bool useSim);
};
