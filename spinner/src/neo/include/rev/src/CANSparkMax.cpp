/*
 * Copyright (c) 2018-2022 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rev/CANSparkMax.h"

#include <fmt/format.h>
#include <hal/CAN.h>
#include <hal/CANAPI.h>

#include "rev/CANSparkMaxDriver.h"

using namespace rev;

constexpr CANSparkMax::ExternalFollower CANSparkMax::kFollowerDisabled;
constexpr CANSparkMax::ExternalFollower CANSparkMax::kFollowerSparkMax;
constexpr CANSparkMax::ExternalFollower CANSparkMax::kFollowerPhoenix;

CANSparkMax::CANSparkMax(int deviceID, MotorType type)
    : CANSparkMaxLowLevel(deviceID, type) {}

void CANSparkMax::Set(double speed) {
    // Only for 'get' API
    m_setpoint = speed;
    SetpointCommand(speed, ControlType::kDutyCycle);
}

void CANSparkMax::SetVoltage(units::volt_t output) {
    // simple conversion too keep Get() trivial
    // Use GetAppliedOutput() instead of Get() for
    // actual applied duty cycle
    double dOutput = units::unit_cast<double>(output);
    m_setpoint = dOutput / 12.0;
    SetpointCommand(dOutput, ControlType::kVoltage);
}

double CANSparkMax::Get() const { return m_setpoint; }

void CANSparkMax::SetInverted(bool isInverted) {
    c_SparkMax_SetInverted(static_cast<c_SparkMax_handle>(m_sparkMaxHandle),
                           isInverted);
}

bool CANSparkMax::GetInverted() const {
    uint8_t inverted;
    c_SparkMax_GetInverted(static_cast<c_SparkMax_handle>(m_sparkMaxHandle),
                           &inverted);
    return inverted ? true : false;
}

void CANSparkMax::Disable() { Set(0); }

void CANSparkMax::StopMotor() { Set(0); }

SparkMaxRelativeEncoder CANSparkMax::GetEncoder(
    SparkMaxRelativeEncoder::Type encoderType, int countsPerRev) {
    bool alreadyCreated = m_relativeEncoderCreated.exchange(true);
    if (alreadyCreated) {
        throw std::runtime_error(fmt::format(
            "GetEncoder() has already been called for SPARK MAX #{}",
            m_deviceID));
    }
    return SparkMaxRelativeEncoder{*this, encoderType, countsPerRev};
}

SparkMaxRelativeEncoder CANSparkMax::GetEncoder(
    CANEncoder::EncoderType encoderType, int countsPerRev) {
    SparkMaxRelativeEncoder::Type newEncoderType;
    if (encoderType == CANEncoder::EncoderType::kNoSensor) {
        newEncoderType = SparkMaxRelativeEncoder::Type::kNoSensor;
    } else if (encoderType == CANEncoder::EncoderType::kHallSensor) {
        newEncoderType = SparkMaxRelativeEncoder::Type::kHallSensor;
    } else if (encoderType == CANEncoder::EncoderType::kQuadrature) {
        newEncoderType = SparkMaxRelativeEncoder::Type::kQuadrature;
    }
    return GetEncoder(newEncoderType, countsPerRev);
}

SparkMaxAlternateEncoder CANSparkMax::GetAlternateEncoder(int countsPerRev) {
    return GetAlternateEncoder(SparkMaxAlternateEncoder::Type::kQuadrature,
                               countsPerRev);
}

SparkMaxAlternateEncoder CANSparkMax::GetAlternateEncoder(
    SparkMaxAlternateEncoder::Type encoderType, int countsPerRev) {
    bool alreadyCreated = m_alternateEncoderCreated.exchange(true);
    if (alreadyCreated) {
        throw std::runtime_error(fmt::format(
            "GetAlternateEncoder() has already been called for SPARK MAX #{}",
            m_deviceID));
    }
    return SparkMaxAlternateEncoder{*this, encoderType, countsPerRev};
}

SparkMaxAlternateEncoder CANSparkMax::GetAlternateEncoder(
    CANEncoder::AlternateEncoderType encoderType, int countsPerRev) {
    return GetAlternateEncoder(SparkMaxAlternateEncoder::Type::kQuadrature,
                               countsPerRev);
}

SparkMaxAnalogSensor CANSparkMax::GetAnalog(SparkMaxAnalogSensor::Mode mode) {
    bool alreadyCreated = m_analogSensorCreated.exchange(true);
    if (alreadyCreated) {
        throw std::runtime_error(
            fmt::format("GetAnalog() has already been called for SPARK MAX #{}",
                        m_deviceID));
    }
    return SparkMaxAnalogSensor{*this, mode};
}

SparkMaxAnalogSensor CANSparkMax::GetAnalog(CANAnalog::AnalogMode mode) {
    SparkMaxAnalogSensor::Mode newMode;
    if (mode == CANAnalog::AnalogMode::kRelative) {
        newMode = SparkMaxAnalogSensor::Mode::kRelative;
    } else {
        newMode = SparkMaxAnalogSensor::Mode::kAbsolute;
    }
    return GetAnalog(newMode);
}

SparkMaxPIDController CANSparkMax::GetPIDController() {
    bool alreadyCreated = m_pidControllerCreated.exchange(true);
    if (alreadyCreated) {
        throw std::runtime_error(fmt::format(
            "GetPIDController() has already been called for SPARK MAX #{}",
            m_deviceID));
    }
    return SparkMaxPIDController{*this};
}

SparkMaxLimitSwitch CANSparkMax::GetForwardLimitSwitch(
    SparkMaxLimitSwitch::Type switchType) {
    bool alreadyCreated = m_forwardLimitSwitchCreated.exchange(true);
    if (alreadyCreated) {
        throw std::runtime_error(fmt::format(
            "GetForwardLimitSwitch() has already been called for SPARK MAX #{}",
            m_deviceID));
    }
    return SparkMaxLimitSwitch{*this, SparkMaxLimitSwitch::Direction::kForward,
                               switchType};
}

SparkMaxLimitSwitch CANSparkMax::GetForwardLimitSwitch(
    CANDigitalInput::LimitSwitchPolarity polarity) {
    SparkMaxLimitSwitch::Type switchType;
    if (polarity == CANDigitalInput::LimitSwitchPolarity::kNormallyOpen) {
        switchType = SparkMaxLimitSwitch::Type::kNormallyOpen;
    } else {
        switchType = SparkMaxLimitSwitch::Type::kNormallyClosed;
    }
    return GetForwardLimitSwitch(switchType);
}

SparkMaxLimitSwitch CANSparkMax::GetReverseLimitSwitch(
    SparkMaxLimitSwitch::Type switchType) {
    bool alreadyCreated = m_reverseLimitSwitchCreated.exchange(true);
    if (alreadyCreated) {
        throw std::runtime_error(fmt::format(
            "GetReverseLimitSwitch() has already been called for SPARK MAX #{}",
            m_deviceID));
    }
    return SparkMaxLimitSwitch{*this, SparkMaxLimitSwitch::Direction::kReverse,
                               switchType};
}

SparkMaxLimitSwitch CANSparkMax::GetReverseLimitSwitch(
    CANDigitalInput::LimitSwitchPolarity polarity) {
    SparkMaxLimitSwitch::Type switchType;
    if (polarity == CANDigitalInput::LimitSwitchPolarity::kNormallyOpen) {
        switchType = SparkMaxLimitSwitch::Type::kNormallyOpen;
    } else {
        switchType = SparkMaxLimitSwitch::Type::kNormallyClosed;
    }
    return GetReverseLimitSwitch(switchType);
}

REVLibError CANSparkMax::SetSmartCurrentLimit(unsigned int limit) {
    return SetSmartCurrentLimit(limit, 0, 20000);
}

REVLibError CANSparkMax::SetSmartCurrentLimit(unsigned int stallLimit,
                                              unsigned int freeLimit,
                                              unsigned int limitRPM) {
    auto status = c_SparkMax_SetSmartCurrentLimit(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), stallLimit, freeLimit,
        limitRPM);
    return static_cast<REVLibError>(status);
}

REVLibError CANSparkMax::SetSecondaryCurrentLimit(double limit,
                                                  int chopCycles) {
    auto status = c_SparkMax_SetSecondaryCurrentLimit(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), limit, chopCycles);
    return static_cast<REVLibError>(status);
}

REVLibError CANSparkMax::SetIdleMode(IdleMode idleMode) {
    auto status =
        c_SparkMax_SetIdleMode(static_cast<c_SparkMax_handle>(m_sparkMaxHandle),
                               static_cast<c_SparkMax_IdleMode>(idleMode));
    return static_cast<REVLibError>(status);
}

CANSparkMax::IdleMode CANSparkMax::GetIdleMode() {
    c_SparkMax_IdleMode idleMode;
    c_SparkMax_GetIdleMode(static_cast<c_SparkMax_handle>(m_sparkMaxHandle),
                           &idleMode);
    return static_cast<CANSparkMax::IdleMode>(idleMode);
}

REVLibError CANSparkMax::EnableVoltageCompensation(double nominalVoltage) {
    auto status = c_SparkMax_EnableVoltageCompensation(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), nominalVoltage);
    return static_cast<REVLibError>(status);
}

REVLibError CANSparkMax::DisableVoltageCompensation() {
    auto status = c_SparkMax_DisableVoltageCompensation(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle));
    return static_cast<REVLibError>(status);
}

double CANSparkMax::GetVoltageCompensationNominalVoltage() {
    float nominalVoltage;
    c_SparkMax_GetVoltageCompensationNominalVoltage(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), &nominalVoltage);
    return static_cast<double>(nominalVoltage);
}

REVLibError CANSparkMax::SetOpenLoopRampRate(double rate) {
    auto status = c_SparkMax_SetOpenLoopRampRate(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), rate);
    return static_cast<REVLibError>(status);
}

REVLibError CANSparkMax::SetClosedLoopRampRate(double rate) {
    auto status = c_SparkMax_SetClosedLoopRampRate(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), rate);
    return static_cast<REVLibError>(status);
}

double CANSparkMax::GetOpenLoopRampRate() {
    float rate;
    c_SparkMax_GetOpenLoopRampRate(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), &rate);
    return static_cast<double>(rate);
}

double CANSparkMax::GetClosedLoopRampRate() {
    float rate;
    c_SparkMax_GetClosedLoopRampRate(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), &rate);
    return static_cast<double>(rate);
}

REVLibError CANSparkMax::Follow(const CANSparkMax& leader, bool invert) {
    return Follow(kFollowerSparkMax, leader.GetDeviceId(), invert);
}

REVLibError CANSparkMax::Follow(ExternalFollower leader, int deviceID,
                                bool invert) {
    FollowConfig maxFollower;
    maxFollower.leaderArbId = (!leader.arbId ? 0 : leader.arbId | deviceID);
    maxFollower.config.bits.predefined = leader.configId;
    maxFollower.config.bits.invert = invert;
    return SetFollow(maxFollower);
}

bool CANSparkMax::IsFollower() {
    uint8_t isFollower;
    c_SparkMax_IsFollower(static_cast<c_SparkMax_handle>(m_sparkMaxHandle),
                          &isFollower);
    return isFollower ? true : false;
}

uint16_t CANSparkMax::GetFaults() {
    uint16_t faults;
    c_SparkMax_GetFaults(static_cast<c_SparkMax_handle>(m_sparkMaxHandle),
                         &faults);
    return faults;
}

uint16_t CANSparkMax::GetStickyFaults() {
    uint16_t faults;
    c_SparkMax_GetStickyFaults(static_cast<c_SparkMax_handle>(m_sparkMaxHandle),
                               &faults);
    return faults;
}

bool CANSparkMax::GetFault(FaultID faultID) const {
    uint8_t fault;
    c_SparkMax_GetFault(static_cast<c_SparkMax_handle>(m_sparkMaxHandle),
                        static_cast<c_SparkMax_FaultID>(faultID), &fault);
    return fault ? true : false;
}

bool CANSparkMax::GetStickyFault(FaultID faultID) const {
    uint8_t fault;
    c_SparkMax_GetStickyFault(static_cast<c_SparkMax_handle>(m_sparkMaxHandle),
                              static_cast<c_SparkMax_FaultID>(faultID), &fault);
    return fault ? true : false;
}

double CANSparkMax::GetBusVoltage() {
    float tmp;
    c_SparkMax_GetBusVoltage(static_cast<c_SparkMax_handle>(m_sparkMaxHandle),
                             &tmp);
    return static_cast<double>(tmp);
}

double CANSparkMax::GetAppliedOutput() {
    float tmp;
    c_SparkMax_GetAppliedOutput(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

double CANSparkMax::GetOutputCurrent() {
    float tmp;
    c_SparkMax_GetOutputCurrent(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

double CANSparkMax::GetMotorTemperature() {
    float tmp;
    c_SparkMax_GetMotorTemperature(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

REVLibError CANSparkMax::ClearFaults() {
    auto status = c_SparkMax_ClearFaults(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle));
    return static_cast<REVLibError>(status);
}

REVLibError CANSparkMax::BurnFlash() {
    auto status =
        c_SparkMax_BurnFlash(static_cast<c_SparkMax_handle>(m_sparkMaxHandle));
    return static_cast<REVLibError>(status);
}

REVLibError CANSparkMax::SetCANTimeout(int milliseconds) {
    auto status = c_SparkMax_SetCANTimeout(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), milliseconds);
    return static_cast<REVLibError>(status);
}

REVLibError CANSparkMax::EnableSoftLimit(SoftLimitDirection direction,
                                         bool enable) {
    c_SparkMax_LimitDirection dir;

    if (direction == SoftLimitDirection::kForward) {
        dir = c_SparkMax_kForward;
    } else {
        dir = c_SparkMax_kReverse;
    }
    auto status = c_SparkMax_EnableSoftLimit(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), dir, enable ? 1 : 0);
    return static_cast<REVLibError>(status);
}

REVLibError CANSparkMax::SetSoftLimit(CANSparkMax::SoftLimitDirection direction,
                                      double limit) {
    c_SparkMax_LimitDirection dir;

    if (direction == SoftLimitDirection::kForward) {
        dir = c_SparkMax_kForward;
    } else {
        dir = c_SparkMax_kReverse;
    }
    auto status = c_SparkMax_SetSoftLimit(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), dir, limit);
    return static_cast<REVLibError>(status);
}

double CANSparkMax::GetSoftLimit(CANSparkMax::SoftLimitDirection direction) {
    c_SparkMax_LimitDirection dir;
    float limit;

    if (direction == SoftLimitDirection::kForward) {
        dir = c_SparkMax_kForward;
    } else {
        dir = c_SparkMax_kReverse;
    }

    c_SparkMax_GetSoftLimit(static_cast<c_SparkMax_handle>(m_sparkMaxHandle),
                            dir, &limit);
    return static_cast<double>(limit);
}

bool CANSparkMax::IsSoftLimitEnabled(
    CANSparkMax::SoftLimitDirection direction) {
    c_SparkMax_LimitDirection dir;

    if (direction == SoftLimitDirection::kForward) {
        dir = c_SparkMax_kForward;
    } else {
        dir = c_SparkMax_kReverse;
    }

    uint8_t enable;
    c_SparkMax_IsSoftLimitEnabled(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), dir, &enable);
    return enable ? true : false;
}

int CANSparkMax::GetFeedbackDeviceID() {
    uint32_t id;
    c_SparkMax_GetFeedbackDeviceID(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), &id);
    return static_cast<int>(id);
}

REVLibError CANSparkMax::GetLastError() {
    return static_cast<REVLibError>(c_SparkMax_GetLastError(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle)));
}

// Used by the HIL tester
SparkMaxRelativeEncoder CANSparkMax::GetEncoderEvenIfAlreadyCreated(
    SparkMaxRelativeEncoder::Type encoderType, int countsPerRev) {
    m_relativeEncoderCreated.exchange(true);
    return SparkMaxRelativeEncoder{*this, encoderType, countsPerRev};
}

REVLibError CANSparkMax::SetSimFreeSpeed(double freeSpeed) {
    return static_cast<REVLibError>(c_SparkMax_SetSimFreeSpeed(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), freeSpeed));
}

REVLibError CANSparkMax::SetSimStallTorque(double stallTorque) {
    return static_cast<REVLibError>(c_SparkMax_SetSimStallTorque(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), stallTorque));
}
