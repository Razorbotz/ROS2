#pragma once

#include "drive_motor_hal.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <messages/msg/falcon_status.hpp>
#include <messages/msg/kraken_status.hpp>
#include <cmath>
#include <algorithm>
#include <map>
#include <string>


/**
 * @brief Simulated drive motor for Gazebo.
 *
 * Bridges between the unified drive node and Gazebo's ros2_control
 * velocity controllers. Converts percent output to velocity commands,
 * reads back joint state feedback, and converts to the sensor units
 * expected by whichever motor type is being emulated.
 *
 * The emulated motor type is set via parameter so the drivetrain
 * node's odometry math gets the correct unit conversion applied.
 */
class DriveMotorSim : public DriveMotorHAL {
public:
    ~DriveMotorSim() override = default;

    bool initialize(rclcpp::Node::SharedPtr node, int motorNumber,
                    const std::string& /*canInterface*/, int /*diagnosticsPort*/,
                    bool /*invertMotor*/, double /*kP*/, double /*kI*/,
                    double /*kD*/, double /*kF*/,
                    double /*supplyCurrentLimit*/) override
    {
        node_ = node;
        motorNumber_ = motorNumber;

        // Determine which motor type we're emulating
        std::string emulateType = "phoenix5";
        try {
            node_->declare_parameter<std::string>("emulate_type", "phoenix5");
        } catch (...) {}
        node_->get_parameter("emulate_type", emulateType);
        emulatedType_ = (emulateType == "phoenix6")
            ? MotorType::KRAKEN_X60
            : MotorType::FALCON_500;

        // Look up Gazebo topic and joint name for this motor
        auto config = getMotorConfig(motorNumber);
        gazeboTopic_ = config.first;
        jointName_ = config.second;

        // Publisher for Gazebo velocity controller
        gazeboPub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            gazeboTopic_, 10);

        // Subscribe to joint states
        jointStateSub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                for (size_t i = 0; i < msg->name.size(); ++i) {
                    if (msg->name[i] == jointName_) {
                        jointPosition_ = msg->position[i];
                        jointVelocity_ = msg->velocity[i];
                        if (i < msg->effort.size())
                            jointEffort_ = msg->effort[i];
                        hasFeedback_ = true;
                        break;
                    }
                }
            });

        RCLCPP_INFO(node_->get_logger(),
                     "DriveMotorSim initialized (ID %d, joint: %s, topic: %s, emulating: %s)",
                     motorNumber, jointName_.c_str(), gazeboTopic_.c_str(),
                     emulateType.c_str());
        return true;
    }

    // --- Control ---

    void setPercentOutput(double percent) override {
        percentOutput_ = std::clamp(percent, -1.0, 1.0);

        // Convert percent to velocity command for Gazebo
        // Max motor speed ~10 rad/s at 100% (tunable)
        double targetVelocity = percent * maxVelRadS_;

        std_msgs::msg::Float64MultiArray cmd;
        cmd.data = {targetVelocity};
        gazeboPub_->publish(cmd);
    }

    void setVelocity(double velocityNative) override {
        // Convert native units to rad/s for Gazebo
        double radS;
        if (emulatedType_ == MotorType::KRAKEN_X60) {
            // Phoenix 6: turns/sec -> rad/s
            radS = velocityNative * 2.0 * M_PI;
        } else {
            // Phoenix 5: units/100ms -> rad/s
            radS = (velocityNative * 10.0 / 2048.0) * 2.0 * M_PI;
        }

        std_msgs::msg::Float64MultiArray cmd;
        cmd.data = {radS};
        gazeboPub_->publish(cmd);
    }

    void feedEnable(int) override {}
    void clearStickyFaults() override {}

    // --- Feedback ---

    int getDeviceID() override { return motorNumber_; }
    double getBusVoltage() override { return 16.0; }

    double getOutputCurrent() override {
        return hasFeedback_ ? std::abs(jointEffort_) * 5.0 : 0.0;
    }

    double getMotorOutputVoltage() override {
        return percentOutput_ * 16.0;
    }

    double getMotorOutputPercent() override {
        return percentOutput_;
    }

    double getTemperature() override { return 45.0; }

    double getSensorPosition() override {
        if (!hasFeedback_) return 0.0;

        if (emulatedType_ == MotorType::KRAKEN_X60) {
            // Phoenix 6: return rotations
            return jointPosition_ / (2.0 * M_PI);
        } else {
            // Phoenix 5: return raw sensor units (2048 per rev)
            return jointPosition_ * (2048.0 / (2.0 * M_PI));
        }
    }

    double getSensorVelocity() override {
        if (!hasFeedback_) return 0.0;

        if (emulatedType_ == MotorType::KRAKEN_X60) {
            // Phoenix 6: return rotations per second
            return jointVelocity_ / (2.0 * M_PI);
        } else {
            // Phoenix 5: return units per 100ms
            return (jointVelocity_ * (2048.0 / (2.0 * M_PI))) / 10.0;
        }
    }

    int getClosedLoopError() override { return 0; }
    double getIntegralAccumulator() override { return 0.0; }
    double getErrorDerivative() override { return 0.0; }
    bool isOvercurrentTripped() override { return false; }

    MotorType getMotorType() override { return emulatedType_; }

    void createStatusPublisher(const std::string& infoTopic) override {
        if (emulatedType_ == MotorType::KRAKEN_X60) {
            krakenPub_ = node_->create_publisher<messages::msg::KrakenStatus>(infoTopic, 1);
        } else {
            falconPub_ = node_->create_publisher<messages::msg::FalconStatus>(infoTopic, 1);
        }
    }

    void publishStatus(bool tempDisable, bool errorFlag,
                       bool restartedFlag, float maxCurrent) override {
        if (emulatedType_ == MotorType::KRAKEN_X60) {
            messages::msg::KrakenStatus status;
            status.device_id       = getDeviceID();
            status.bus_voltage     = getBusVoltage();
            status.output_current  = getOutputCurrent();
            status.output_voltage  = getMotorOutputVoltage();
            status.output_percent  = getMotorOutputPercent();
            status.temperature     = getTemperature();
            status.sensor_position = getSensorPosition();
            status.sensor_velocity = getSensorVelocity();
            status.closed_loop_error = getClosedLoopError();
            status.temp_disable    = tempDisable;
            status.error           = errorFlag;
            status.restarted       = restartedFlag;
            status.max_current     = maxCurrent;
            krakenPub_->publish(status);
        } else {
            messages::msg::FalconStatus status;
            status.device_id            = getDeviceID();
            status.bus_voltage          = getBusVoltage();
            status.output_current       = getOutputCurrent();
            status.output_voltage       = getMotorOutputVoltage();
            status.output_percent       = getMotorOutputPercent();
            status.temperature          = getTemperature();
            status.sensor_position      = getSensorPosition();
            status.sensor_velocity      = getSensorVelocity();
            status.closed_loop_error    = getClosedLoopError();
            status.integral_accumulator = getIntegralAccumulator();
            status.error_derivative     = getErrorDerivative();
            status.temp_disable         = tempDisable;
            status.error                = errorFlag;
            status.restarted            = restartedFlag;
            status.max_current          = maxCurrent;
            falconPub_->publish(status);
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    int motorNumber_ = 0;
    MotorType emulatedType_ = MotorType::FALCON_500;

    std::string gazeboTopic_;
    std::string jointName_;
    double maxVelRadS_ = 10.0;

    double percentOutput_ = 0.0;
    double jointPosition_ = 0.0;
    double jointVelocity_ = 0.0;
    double jointEffort_ = 0.0;
    bool hasFeedback_ = false;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gazeboPub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
    std::shared_ptr<rclcpp::Publisher<messages::msg::FalconStatus>> falconPub_;
    std::shared_ptr<rclcpp::Publisher<messages::msg::KrakenStatus>> krakenPub_;

    /**
     * @brief Map motor ID to Gazebo controller topic and joint name.
     *        Matches your existing falcon_sim_node mapping.
     */
    static std::pair<std::string, std::string> getMotorConfig(int motorNumber) {
        static const std::map<int, std::pair<std::string, std::string>> configs = {
            {10, {"/falcon_10_controller/commands", "FR_Wheel_Joint"}},
            {11, {"/falcon_11_controller/commands", "FL_Wheel_Joint"}},
            {12, {"/falcon_12_controller/commands", "BR_Wheel_Joint"}},
            {13, {"/falcon_13_controller/commands", "BL_Wheel_Joint"}},
        };

        auto it = configs.find(motorNumber);
        if (it != configs.end()) {
            return it->second;
        }

        // Fallback
        return {"/motor_" + std::to_string(motorNumber) + "_controller/commands",
                "wheel_joint_" + std::to_string(motorNumber)};
    }
};