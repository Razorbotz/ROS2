#pragma once

#include "drive_motor_hal.hpp"

#include <cstring>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <thread>
#include <chrono>
#include <unistd.h>

// Phoenix 6 Includes
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/VelocityDutyCycle.hpp>
#include <ctre/phoenix/unmanaged/Unmanaged.h>
#include <units/current.h>

#include <messages/msg/kraken_status.hpp>

// Use specific namespaces to avoid collisions with Phoenix 5's ctre::phoenix
namespace ph6 = ctre::phoenix6;


/**
 * @brief Kraken x60 implementation via Phoenix 6 TalonFX API.
 *
 * Sensor units (Phoenix 6 native):
 *   - Position: rotations (motor shaft)
 *   - Velocity: rotations per second (motor shaft)
 *
 * This is different from Phoenix 5's raw sensor units. The drivetrain
 * node handles the conversion difference via the MotorType identifier.
 */
class KrakenHardware : public DriveMotorHAL {
public:
    ~KrakenHardware() override {
        delete talonFX_;
    }

    bool initialize(rclcpp::Node::SharedPtr node, int motorNumber,
                    const std::string& canInterface, int diagnosticsPort,
                    bool invertMotor, double kP, double kI,
                    double kD, double kF,
                    double supplyCurrentLimit) override
    {
        node_ = node;
        motorNumber_ = motorNumber;

        // Wait for CAN bus
        while (rclcpp::ok() && !canSocketBindOk(canInterface)) {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "CAN interface '%s' is DOWN / not bindable. Waiting...",
                canInterface.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // Set diagnostics port via environment variable (Phoenix 6 pattern)
        setenv("PHOENIX_DIAGNOSTICS_PORT", std::to_string(diagnosticsPort).c_str(), 1);

        RCLCPP_INFO(node_->get_logger(), "Opened CAN interface: %s", canInterface.c_str());

        // Phoenix 6 2026: use CANBus object constructor
        talonFX_ = new ph6::hardware::TalonFX(motorNumber, ph6::CANBus{canInterface});
        RCLCPP_INFO(node_->get_logger(), "Created Kraken x60 TalonFX (ID %d)", motorNumber);

        ph6::configs::TalonFXConfiguration allConfigs;
        allConfigs.Slot0.kP = kP;
        allConfigs.Slot0.kI = kI;
        allConfigs.Slot0.kD = kD;
        allConfigs.Slot0.kS = kF;

        allConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        allConfigs.CurrentLimits.SupplyCurrentLimit =
            units::current::ampere_t{supplyCurrentLimit};

        if (invertMotor) {
            allConfigs.MotorOutput.Inverted =
                ph6::signals::InvertedValue::CounterClockwise_Positive;
        } else {
            allConfigs.MotorOutput.Inverted =
                ph6::signals::InvertedValue::Clockwise_Positive;
        }

        talonFX_->GetConfigurator().Apply(allConfigs);
        talonFX_->SetControl(percentOut_.WithOutput(0.0));

        RCLCPP_INFO(node_->get_logger(), "Kraken x60 configured (current limit: %.0fA)",
                     supplyCurrentLimit);
        return true;
    }

    void setPercentOutput(double percent) override {
        talonFX_->SetControl(percentOut_.WithOutput(percent));
        lastPercent_ = percent;
    }

    void setVelocity(double velocityNative) override {
        // Phoenix 6: velocity in turns per second
        talonFX_->SetControl(velOut_.WithVelocity(
            units::angular_velocity::turns_per_second_t{velocityNative}));
    }

    void feedEnable(int timeoutMs) override {
        ctre::phoenix::unmanaged::FeedEnable(timeoutMs);
    }

    void clearStickyFaults() override {
        talonFX_->ClearStickyFaults();
    }

    int getDeviceID() override {
        return talonFX_->GetDeviceID();
    }

    double getBusVoltage() override {
        return talonFX_->GetSupplyVoltage().GetValueAsDouble();
    }

    double getOutputCurrent() override {
        return talonFX_->GetStatorCurrent().GetValueAsDouble();
    }

    double getMotorOutputVoltage() override {
        return talonFX_->GetMotorVoltage().GetValueAsDouble();
    }

    double getMotorOutputPercent() override {
        return talonFX_->GetDutyCycle().GetValueAsDouble();
    }

    double getTemperature() override {
        return talonFX_->GetDeviceTemp().GetValueAsDouble();
    }

    double getSensorPosition() override {
        // Phoenix 6: returns rotations
        return talonFX_->GetPosition().GetValueAsDouble();
    }

    double getSensorVelocity() override {
        // Phoenix 6: returns rotations per second
        return talonFX_->GetVelocity().GetValueAsDouble();
    }

    int getClosedLoopError() override {
        return static_cast<int>(talonFX_->GetClosedLoopError().GetValueAsDouble());
    }

    // Phoenix 6 doesn't expose these the same way as Phoenix 5
    double getIntegralAccumulator() override { return 0.0; }
    double getErrorDerivative() override     { return 0.0; }

    bool isOvercurrentTripped() override {
        bool supplyCurrent = talonFX_->GetStickyFault_SupplyCurrLimit().GetValue();
        bool statorCurrent = talonFX_->GetStickyFault_StatorCurrLimit().GetValue();
        bool procTemp      = talonFX_->GetStickyFault_ProcTemp().GetValue();
        return supplyCurrent || statorCurrent || procTemp;
    }

    MotorType getMotorType() override { return MotorType::KRAKEN_X60; }

    void createStatusPublisher(const std::string& infoTopic) override {
        statusPub_ = node_->create_publisher<messages::msg::KrakenStatus>(infoTopic, 1);
    }

    void publishStatus(bool tempDisable, bool errorFlag,
                       bool restartedFlag, float maxCurrent) override {
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
        statusPub_->publish(status);
    }

private:
    rclcpp::Node::SharedPtr node_;
    ph6::hardware::TalonFX* talonFX_ = nullptr;
    int motorNumber_ = 0;
    double lastPercent_ = 0.0;
    std::shared_ptr<rclcpp::Publisher<messages::msg::KrakenStatus>> statusPub_;

    ph6::controls::DutyCycleOut percentOut_{0.0};
    ph6::controls::VelocityDutyCycle velOut_{units::angular_velocity::turns_per_second_t{0}};

    static bool canSocketBindOk(const std::string& ifname) {
        int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0) return false;
        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
        if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { close(s); return false; }
        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        bool ok = (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
        close(s);
        return ok;
    }
};