#pragma once

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>


/**
 * @brief Hardware abstraction layer for drive motors (Falcon 500 / Kraken x60).
 *
 * Provides a uniform interface for physical hardware and Gazebo simulation.
 * Covers both Phoenix 5 (Falcon 500 via TalonFX) and Phoenix 6 (Kraken x60
 * via TalonFX) through the same abstract interface.
 *
 * Each backend creates its own status publisher (FalconStatus or KrakenStatus)
 * and populates it with the correct message type and field names. The unified
 * node just calls publishStatus() without caring which message type is used.
 */
class DriveMotorHAL {
public:
    virtual ~DriveMotorHAL() = default;

    /**
     * @brief Motor type identifier — used by the drivetrain node to
     *        know which unit conversions to apply for odometry.
     */
    enum class MotorType {
        FALCON_500,     // Phoenix 5: 2048 units/rev, velocity in units/100ms
        KRAKEN_X60,     // Phoenix 6: rotations/sec, position in rotations
        SIMULATED       // Gazebo: matches whichever type is being emulated
    };

    // --- Lifecycle ---
    virtual bool initialize(rclcpp::Node::SharedPtr node, int motorNumber,
                            const std::string& canInterface, int diagnosticsPort,
                            bool invertMotor, double kP, double kI,
                            double kD, double kF,
                            double supplyCurrentLimit) = 0;

    /**
     * @brief Create the status publisher for this motor type.
     *        Must be called after initialize().
     * @param infoTopic  Topic name for the status message (e.g. "talon_10_info")
     */
    virtual void createStatusPublisher(const std::string& infoTopic) = 0;

    /**
     * @brief Read sensor data and publish the motor-specific status message.
     *        Falcon backends publish FalconStatus, Kraken backends publish
     *        KrakenStatus. Call this from the main loop at your desired rate.
     * @param tempDisable  Whether the motor is thermally disabled
     * @param error        Whether an overcurrent fault is active
     * @param restarted    Whether the motor has recovered from a fault
     * @param maxCurrent   Peak current seen so far
     */
    virtual void publishStatus(bool tempDisable, bool error,
                               bool restarted, float maxCurrent) = 0;

    // --- Control ---
    virtual void setPercentOutput(double percent) = 0;
    virtual void setVelocity(double velocityNative) = 0;
    virtual void feedEnable(int timeoutMs) = 0;
    virtual void clearStickyFaults() = 0;

    // --- Feedback ---
    virtual int    getDeviceID() = 0;
    virtual double getBusVoltage() = 0;
    virtual double getOutputCurrent() = 0;
    virtual double getMotorOutputVoltage() = 0;
    virtual double getMotorOutputPercent() = 0;
    virtual double getTemperature() = 0;
    virtual double getSensorPosition() = 0;
    virtual double getSensorVelocity() = 0;
    virtual int    getClosedLoopError() = 0;
    virtual double getIntegralAccumulator() = 0;
    virtual double getErrorDerivative() = 0;

    // --- Fault detection ---
    virtual bool isOvercurrentTripped() = 0;

    // --- Identity ---
    virtual MotorType getMotorType() = 0;
};