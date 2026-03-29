#pragma once

#include "talon_hal.hpp"

#include <cstring>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <thread>
#include <chrono>
#include <unistd.h>

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
 * @brief Physical TalonSRX implementation via Phoenix 5 API.
 *
 * This is the real hardware path. It configures the TalonSRX over CAN,
 * reads actual sensor values, and drives the physical motor.
 */
class TalonHardware : public TalonHAL {
public:
    ~TalonHardware() override {
        delete talonSRX_;
    }

    bool initialize(rclcpp::Node::SharedPtr node, int motorNumber,
                    const std::string& canInterface, int diagnosticsPort,
                    bool invertMotor, double kP, double kI,
                    double kD, double kF) override
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

        c_Phoenix_Diagnostics_Create1(diagnosticsPort);

        const int kTimeoutMs = 30;
        const int kPIDLoopIdx = 0;

        talonSRX_ = new TalonSRX(motorNumber);
        RCLCPP_INFO(node_->get_logger(), "Created TalonSRX instance (ID %d)", motorNumber);

        talonSRX_->SetInverted(invertMotor);
        talonSRX_->SelectProfileSlot(0, 0);
        talonSRX_->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, kTimeoutMs);
        talonSRX_->SetSensorPhase(true);
        talonSRX_->ConfigClosedloopRamp(2);
        talonSRX_->ConfigNominalOutputForward(0, kTimeoutMs);
        talonSRX_->ConfigNominalOutputReverse(0, kTimeoutMs);
        talonSRX_->ConfigPeakOutputForward(1, kTimeoutMs);
        talonSRX_->ConfigPeakOutputReverse(-1, kTimeoutMs);
        talonSRX_->Config_kF(kPIDLoopIdx, kF, kTimeoutMs);
        talonSRX_->Config_kP(kPIDLoopIdx, kP, kTimeoutMs);
        talonSRX_->Config_kI(kPIDLoopIdx, kI, kTimeoutMs);
        talonSRX_->Config_kD(kPIDLoopIdx, kD, kTimeoutMs);
        talonSRX_->ConfigAllowableClosedloopError(kPIDLoopIdx, 0, kTimeoutMs);

        talonSRX_->SetControlFramePeriod(ControlFrame::Control_3_General, 20);
        talonSRX_->SetControlFramePeriod(ControlFrame::Control_4_Advanced, 20);
        talonSRX_->Set(ControlMode::PercentOutput, 0);
        talonSRX_->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 10, 10);

        RCLCPP_INFO(node_->get_logger(), "TalonSRX configured");
        return true;
    }

    void setPercentOutput(double percent) override {
        talonSRX_->Set(ControlMode::PercentOutput, percent);
    }

    void setPosition(int position) override {
        talonSRX_->Set(ControlMode::Position, position);
    }

    void feedEnable(int timeoutMs) override {
        ctre::phoenix::unmanaged::FeedEnable(timeoutMs);
    }

    int    getDeviceID() override          { return talonSRX_->GetDeviceID(); }
    double getBusVoltage() override        { return talonSRX_->GetBusVoltage(); }
    double getOutputCurrent() override     { return talonSRX_->GetOutputCurrent(); }
    double getMotorOutputVoltage() override{ return talonSRX_->GetMotorOutputVoltage(); }
    double getMotorOutputPercent() override{ return talonSRX_->GetMotorOutputPercent(); }
    double getTemperature() override       { return talonSRX_->GetTemperature(); }
    int    getSensorPosition() override    { return talonSRX_->GetSelectedSensorPosition(0); }
    double getSensorVelocity() override    { return talonSRX_->GetSelectedSensorVelocity(0); }
    int    getClosedLoopError() override   { return talonSRX_->GetClosedLoopError(0); }
    double getIntegralAccumulator() override { return talonSRX_->GetIntegralAccumulator(0); }
    double getErrorDerivative() override   { return talonSRX_->GetErrorDerivative(0); }

private:
    rclcpp::Node::SharedPtr node_;
    TalonSRX* talonSRX_ = nullptr;
    int motorNumber_ = 0;

    static bool canSocketBindOk(const std::string& ifname) {
        int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0) return false;

        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);

        if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
            close(s);
            return false;
        }

        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        bool ok = (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
        close(s);
        return ok;
    }
};
