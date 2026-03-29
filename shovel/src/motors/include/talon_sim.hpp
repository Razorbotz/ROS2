#pragma once

#include "talon_hal.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <algorithm>
#include <map>
#include <string>


/**
 * @brief Simulated TalonSRX implementation for Gazebo.
 *
 * Bridges to Gazebo's ros2_control position controllers. Converts
 * percent output commands into joint angle deltas (emulating linear
 * actuator behavior) and reads back joint state feedback, converting
 * joint angles to simulated potentiometer values.
 *
 * From the perspective of the unified TalonNode, this behaves
 * identically to the hardware implementation.
 */
class TalonSim : public TalonHAL {
public:
    /**
     * @brief Actuator configuration — maps between joint angles,
     *        linear actuator stroke, and potentiometer values.
     */
    struct ActuatorConfig {
        std::string jointName;          // Joint name in /joint_states
        std::string gazeboTopic;        // ros2_control command topic
        double lowerRad;                // Joint lower limit (radians)
        double upperRad;                // Joint upper limit (radians)
        double strokeIn;                // Actuator stroke (inches)
        double speedInPerS;             // Max actuator speed (inches/sec)
        bool invertPot;                 // Pot direction relative to angle
    };

    ~TalonSim() override = default;

    bool initialize(rclcpp::Node::SharedPtr node, int motorNumber,
                    const std::string& /*canInterface*/, int /*diagnosticsPort*/,
                    bool /*invertMotor*/, double /*kP*/, double /*kI*/,
                    double /*kD*/, double /*kF*/) override
    {
        node_ = node;
        motorNumber_ = motorNumber;

        // Look up actuator config from parameters
        config_ = getActuatorConfig(motorNumber);

        // Publisher for Gazebo position controller
        gazeboPub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            config_.gazeboTopic, 10);

        // Subscribe to joint states for feedback
        jointStateSub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                jointStateCallback(msg);
            });

        // Start update timer (50 Hz)
        lastUpdateTime_ = node_->now();
        updateTimer_ = node_->create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() { update(); });

        RCLCPP_INFO(node_->get_logger(),
                     "TalonSim initialized (ID %d, joint: %s, topic: %s)",
                     motorNumber, config_.jointName.c_str(),
                     config_.gazeboTopic.c_str());
        return true;
    }

    // --- Control ---

    void setPercentOutput(double percent) override {
        percentOutput_ = std::clamp(percent, -1.0, 1.0);
        usePosition_ = false;
    }

    void setPosition(int position) override {
        targetPosition_ = std::clamp(position, 20, 950);
        usePosition_ = true;
    }

    void feedEnable(int /*timeoutMs*/) override {
        // No-op in simulation — motors are always enabled
    }

    // --- Feedback ---

    int    getDeviceID() override           { return motorNumber_; }
    double getBusVoltage() override         { return 16.0; }
    double getOutputCurrent() override      { return (std::abs(percentOutput_) > 0.01) ? 1.0 : 0.0; }
    double getMotorOutputVoltage() override { return percentOutput_ * 16.0; }
    double getMotorOutputPercent() override { return percentOutput_; }
    double getTemperature() override        { return 45.0; }
    int    getSensorPosition() override     { return currentPotValue_; }
    double getSensorVelocity() override     { return 0.0; }
    int    getClosedLoopError() override    { return usePosition_ ? (targetPosition_ - currentPotValue_) : 0; }
    double getIntegralAccumulator() override { return 0.0; }
    double getErrorDerivative() override    { return 0.0; }

private:
    rclcpp::Node::SharedPtr node_;
    int motorNumber_ = 0;
    ActuatorConfig config_;

    // State
    double cmdPosRad_ = 0.0;           // Current commanded joint angle
    double jointFeedbackRad_ = 0.0;    // Feedback from /joint_states
    bool hasFeedback_ = false;
    double percentOutput_ = 0.0;
    int targetPosition_ = 500;
    bool usePosition_ = false;
    int currentPotValue_ = 500;

    rclcpp::Time lastUpdateTime_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gazeboPub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
    rclcpp::TimerBase::SharedPtr updateTimer_;


    /**
     * @brief Returns actuator configuration for a given motor ID.
     *        Add new actuators here as needed.
     */
    static ActuatorConfig getActuatorConfig(int motorNumber) {
        // Default configs for known actuators
        static const std::map<int, ActuatorConfig> configs = {
            {14, {"Arm_Joint",    "/arm_position_controller/commands",
                  -0.6, 0.3, 10.0, 0.5, true}},
            {15, {"Arm_Joint",    "/arm_position_controller/commands",
                  -0.6, 0.3, 10.0, 0.5, true}},
            {16, {"Bucket_Joint", "/bucket_position_controller/commands",
                  -1.25, 0.45, 4.0, 0.5, false}},
            {17, {"Bucket_Joint", "/bucket_position_controller/commands",
                  -1.25, 0.45, 4.0, 0.5, false}},
        };

        auto it = configs.find(motorNumber);
        if (it != configs.end()) {
            return it->second;
        }

        // Fallback — generic config
        return {"unknown_joint", "/unknown_controller/commands",
                -1.0, 1.0, 10.0, 0.5, false};
    }


    /**
     * @brief Convert joint angle to potentiometer value (0-1024 range,
     *        working range 20-950).
     */
    int angleToPot(double angleRad) const {
        const double range = config_.upperRad - config_.lowerRad;
        double u = 0.0;
        if (std::abs(range) > 1e-9) {
            u = (angleRad - config_.lowerRad) / range;
        }
        u = std::clamp(u, 0.0, 1.0);

        double potF;
        if (config_.invertPot) {
            potF = 950.0 - u * (950.0 - 20.0);
        } else {
            potF = 20.0 + u * (950.0 - 20.0);
        }

        return std::clamp(static_cast<int>(std::lround(potF)), 0, 1024);
    }


    /**
     * @brief Convert radians per inch of actuator travel.
     */
    double radPerIn() const {
        const double range = config_.upperRad - config_.lowerRad;
        return (config_.strokeIn > 1e-9) ? (range / config_.strokeIn) : 0.0;
    }


    /**
     * @brief Read joint feedback from /joint_states.
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == config_.jointName) {
                jointFeedbackRad_ = msg->position[i];
                hasFeedback_ = true;
                break;
            }
        }
    }


    /**
     * @brief Periodic update — runs position control loop if in position mode,
     *        integrates percent output into joint angle, publishes to Gazebo,
     *        and updates potentiometer feedback.
     */
    void update() {
        const rclcpp::Time t = node_->now();
        double dt = (t - lastUpdateTime_).seconds();
        if (dt <= 0.0) dt = 0.02;
        lastUpdateTime_ = t;

        // If in position mode, compute percent output from error
        if (usePosition_) {
            const double currentAngle = hasFeedback_ ? jointFeedbackRad_ : cmdPosRad_;
            currentPotValue_ = angleToPot(currentAngle);

            int error = targetPosition_ - currentPotValue_;
            percentOutput_ = percentFromError(error, config_.invertPot);
        }

        // Integrate percent output into commanded joint angle
        const double vInS = percentOutput_ * config_.speedInPerS;
        const double wRadS = vInS * radPerIn();
        double next = cmdPosRad_ + wRadS * dt;
        next = std::clamp(next, config_.lowerRad, config_.upperRad);
        cmdPosRad_ = next;

        // Publish to Gazebo
        std_msgs::msg::Float64MultiArray cmd;
        cmd.data = {cmdPosRad_};
        gazeboPub_->publish(cmd);

        // Update pot value from feedback (or command if no feedback yet)
        const double angleForPot = hasFeedback_ ? jointFeedbackRad_ : cmdPosRad_;
        currentPotValue_ = angleToPot(angleForPot);
    }


    /**
     * @brief Simple proportional control mapping error to percent output,
     *        with deadband and speed ramping. Matches the behavior from
     *        the original talon_sim_node.
     */
    static double percentFromError(int error, bool invert) {
        const int mag = std::abs(error);
        double output;

        if (mag <= 5)       output = 0.0;
        else if (mag <= 20) output = (error > 0) ? 0.5  : -0.5;
        else if (mag <= 50) output = (error > 0) ? 0.75 : -0.75;
        else                output = (error > 0) ? 1.0  : -1.0;

        return invert ? -output : output;
    }
};
