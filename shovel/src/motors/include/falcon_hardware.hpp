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

#include <messages/msg/falcon_status.hpp>

#define Phoenix_No_WPI
#include <ctre/Phoenix.h>
#include <ctre/phoenix/platform/Platform.h>
#include <ctre/phoenix/unmanaged/Unmanaged.h>
#include <ctre/phoenix/cci/Unmanaged_CCI.h>
#include <ctre/phoenix/cci/Diagnostics_CCI.h>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


/**
 * @brief Falcon 500 implementation via Phoenix 5 TalonFX API.
 *
 * Sensor units:
 *   - Position: raw integrated sensor units (2048 per revolution)
 *   - Velocity: raw units per 100ms
 */
class FalconHardware : public DriveMotorHAL {
public:
    ~FalconHardware() override {
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

        ctre::phoenix::platform::can::SetCANInterface(canInterface.c_str());
        RCLCPP_INFO(node_->get_logger(), "Opened CAN interface: %s", canInterface.c_str());

        setenv("PHOENIX_DIAGNOSTICS_PORT", std::to_string(diagnosticsPort).c_str(), 1);
        c_Phoenix_Diagnostics_Create();

        const int kTimeoutMs = 30;
        const int kPIDLoopIdx = 0;

        talonFX_ = new TalonFX(motorNumber);
        RCLCPP_INFO(node_->get_logger(), "Created Falcon 500 TalonFX (ID %d)", motorNumber);

        if (invertMotor) {
            talonFX_->SetInverted(TalonFXInvertType::CounterClockwise);
        } else {
            talonFX_->SetInverted(TalonFXInvertType::Clockwise);
        }

        talonFX_->SelectProfileSlot(0, 0);
        talonFX_->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
        talonFX_->ConfigNominalOutputForward(0, kTimeoutMs);
        talonFX_->ConfigNominalOutputReverse(0, kTimeoutMs);
        talonFX_->ConfigPeakOutputForward(1, kTimeoutMs);
        talonFX_->ConfigPeakOutputReverse(-1, kTimeoutMs);
        talonFX_->Config_kF(kPIDLoopIdx, kF, kTimeoutMs);
        talonFX_->Config_kP(kPIDLoopIdx, kP, kTimeoutMs);
        talonFX_->Config_kI(kPIDLoopIdx, kI, kTimeoutMs);
        talonFX_->Config_kD(kPIDLoopIdx, kD, kTimeoutMs);
        talonFX_->ConfigAllowableClosedloopError(kPIDLoopIdx, 0, kTimeoutMs);

        talonFX_->SetControlFramePeriod(ControlFrame::Control_3_General, 20);
        talonFX_->SetControlFramePeriod(ControlFrame::Control_4_Advanced, 20);
        talonFX_->Set(ControlMode::PercentOutput, 0);

        // Supply current limit
        ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration supplyLimitConfig;
        supplyLimitConfig.enable = true;
        supplyLimitConfig.currentLimit = supplyCurrentLimit;
        supplyLimitConfig.triggerThresholdCurrent = supplyCurrentLimit + 5.0;
        supplyLimitConfig.triggerThresholdTime = 0.1;
        talonFX_->ConfigSupplyCurrentLimit(supplyLimitConfig, kTimeoutMs);

        talonFX_->ClearStickyFaults(kTimeoutMs);

        RCLCPP_INFO(node_->get_logger(), "Falcon 500 configured (current limit: %.0fA)",
                     supplyCurrentLimit);
        return true;
    }

    void setPercentOutput(double percent) override {
        talonFX_->Set(ControlMode::PercentOutput, percent);
    }

    void setVelocity(double velocityNative) override {
        talonFX_->Set(ControlMode::Velocity, velocityNative);
    }

    void feedEnable(int timeoutMs) override {
        ctre::phoenix::unmanaged::FeedEnable(timeoutMs);
    }

    void clearStickyFaults() override {
        talonFX_->ClearStickyFaults(30);
    }

    int    getDeviceID() override           { return talonFX_->GetDeviceID(); }
    double getBusVoltage() override         { return talonFX_->GetBusVoltage(); }
    double getOutputCurrent() override      { return talonFX_->GetOutputCurrent(); }
    double getMotorOutputVoltage() override { return talonFX_->GetMotorOutputVoltage(); }
    double getMotorOutputPercent() override { return talonFX_->GetMotorOutputPercent(); }
    double getTemperature() override        { return talonFX_->GetTemperature(); }
    double getSensorPosition() override     { return talonFX_->GetSelectedSensorPosition(0); }
    double getSensorVelocity() override     { return talonFX_->GetSelectedSensorVelocity(0); }
    int    getClosedLoopError() override    { return talonFX_->GetClosedLoopError(0); }
    double getIntegralAccumulator() override { return talonFX_->GetIntegralAccumulator(0); }
    double getErrorDerivative() override    { return talonFX_->GetErrorDerivative(0); }

    bool isOvercurrentTripped() override {
        StickyFaults faults;
        talonFX_->GetStickyFaults(faults);
        return faults.ResetDuringEn || faults.SupplyOverV || faults.SupplyUnstable;
    }

    MotorType getMotorType() override { return MotorType::FALCON_500; }

    void createStatusPublisher(const std::string& infoTopic) override {
        statusPub_ = node_->create_publisher<messages::msg::FalconStatus>(infoTopic, 1);
    }

    void publishStatus(bool tempDisable, bool errorFlag,
                       bool restartedFlag, float maxCurrent) override {
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
        statusPub_->publish(status);
    }

private:
    rclcpp::Node::SharedPtr node_;
    TalonFX* talonFX_ = nullptr;
    int motorNumber_ = 0;
    std::shared_ptr<rclcpp::Publisher<messages::msg::FalconStatus>> statusPub_;

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