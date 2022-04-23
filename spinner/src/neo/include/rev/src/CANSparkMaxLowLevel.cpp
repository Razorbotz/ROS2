/*
 * Copyright (c) 2018-2021 REV Robotics
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

#include "rev/CANSparkMaxLowLevel.h"

#include <cstring>
#include <stdexcept>

#include <fmt/format.h>

#include "rev/CANSparkMaxDriver.h"
#include "rev/CANSparkMaxHeartbeat.h"

namespace rev {

const uint8_t CANSparkMaxLowLevel::kAPIMajorVersion =
    c_SparkMax_kAPIMajorVersion;
const uint8_t CANSparkMaxLowLevel::kAPIMinorVersion =
    c_SparkMax_kAPIMinorVersion;
const uint8_t CANSparkMaxLowLevel::kAPIBuildVersion =
    c_SparkMax_kAPIBuildVersion;
const uint32_t CANSparkMaxLowLevel::kAPIVersion = c_SparkMax_kAPIVersion;

CANSparkMaxLowLevel::CANSparkMaxLowLevel(int deviceID, MotorType type)
    : m_motorType(type), m_deviceID(deviceID) {
    if (c_SparkMax_RegisterId(deviceID) == c_REVLibError_DuplicateCANId) {
        throw std::runtime_error(fmt::format(
            "A CANSparkMax instance has already been created with this device "
            "ID: {}",
            deviceID));
    }
    m_sparkMaxHandle = static_cast<void*>(
        c_SparkMax_Create(deviceID, static_cast<c_SparkMax_MotorType>(type)));
}

CANSparkMaxLowLevel::~CANSparkMaxLowLevel() {
    c_SparkMax_Destroy(static_cast<c_SparkMax_handle>(m_sparkMaxHandle));
}

uint32_t CANSparkMaxLowLevel::GetFirmwareVersion() {
    bool isDebugBuild;
    return GetFirmwareVersion(isDebugBuild);
}

uint32_t CANSparkMaxLowLevel::GetFirmwareVersion(bool& isDebugBuild) {
    c_SparkMax_FirmwareVersion fwVersion;

    c_SparkMax_GetFirmwareVersion(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), &fwVersion);

    isDebugBuild = fwVersion.isDebug;
    return fwVersion.versionRaw;
}

std::string CANSparkMaxLowLevel::GetFirmwareString() {
    c_SparkMax_FirmwareVersion fwVersion;

    c_SparkMax_GetFirmwareVersion(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), &fwVersion);

    if (fwVersion.isDebug) {
        return fmt::format("v{}.{}.{} Debug Build", fwVersion.major,
                           fwVersion.minor, fwVersion.build);
    } else {
        return fmt::format("v{}.{}.{}", fwVersion.major, fwVersion.minor,
                           fwVersion.build);
    }
}

std::vector<uint8_t> CANSparkMaxLowLevel::GetSerialNumber() { return {}; }

int CANSparkMaxLowLevel::GetDeviceId() const { return m_deviceID; }

CANSparkMaxLowLevel::MotorType CANSparkMaxLowLevel::GetInitialMotorType() {
    return m_motorType;
}

CANSparkMaxLowLevel::MotorType CANSparkMaxLowLevel::GetMotorType() {
    return m_motorType;
}

REVLibError CANSparkMaxLowLevel::SetPeriodicFramePeriod(PeriodicFrame frameID,
                                                        int periodMs) {
    auto status = c_SparkMax_SetPeriodicFramePeriod(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle),
        static_cast<c_SparkMax_PeriodicFrame>(frameID), periodMs);

    return static_cast<REVLibError>(status);
}

void CANSparkMaxLowLevel::SetControlFramePeriodMs(int periodMs) {
    c_SparkMax_SetControlFramePeriod(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), periodMs);
}

CANSparkMaxLowLevel::PeriodicStatus0 CANSparkMaxLowLevel::GetPeriodicStatus0() {
    c_SparkMax_PeriodicStatus0 cStatus0;
    c_SparkMax_GetPeriodicStatus0(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), &cStatus0);

    PeriodicStatus0 status0;
    status0.appliedOutput = cStatus0.appliedOutput;
    status0.faults = cStatus0.faults;
    status0.lock = cStatus0.lock;
    status0.roboRIO = cStatus0.roboRIO;
    status0.isInverted = cStatus0.isInverted;
    status0.isFollower = cStatus0.isFollower;
    status0.motorType = static_cast<MotorType>(cStatus0.motorType);
    status0.stickyFaults = cStatus0.stickyFaults;
    status0.timestamp = cStatus0.timestamp;

    return status0;
}

static inline float unpackFloat32ToInt32(int32_t val) {
    float f;
    std::memcpy(&f, &val, sizeof(f));
    return f;
}

CANSparkMaxLowLevel::PeriodicStatus1 CANSparkMaxLowLevel::GetPeriodicStatus1() {
    c_SparkMax_PeriodicStatus1 cStatus1;
    c_SparkMax_GetPeriodicStatus1(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), &cStatus1);

    PeriodicStatus1 status1;
    status1.outputCurrent = cStatus1.outputCurrent;
    status1.busVoltage = cStatus1.busVoltage;
    status1.motorTemperature = cStatus1.motorTemperature;
    status1.sensorVelocity = cStatus1.sensorVelocity;
    status1.timestamp = cStatus1.timestamp;

    return status1;
}

CANSparkMaxLowLevel::PeriodicStatus2 CANSparkMaxLowLevel::GetPeriodicStatus2() {
    c_SparkMax_PeriodicStatus2 cStatus2;
    c_SparkMax_GetPeriodicStatus2(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), &cStatus2);

    PeriodicStatus2 status2;
    status2.sensorPosition = cStatus2.sensorPosition;
    status2.iAccum = cStatus2.iAccum;
    status2.timestamp = cStatus2.timestamp;

    return status2;
}

REVLibError CANSparkMaxLowLevel::SetFollow(FollowConfig follower) {
    auto status =
        c_SparkMax_SetFollow(static_cast<c_SparkMax_handle>(m_sparkMaxHandle),
                             follower.leaderArbId, follower.config.value);
    return static_cast<REVLibError>(status);
}

REVLibError CANSparkMaxLowLevel::SetpointCommand(double value, ControlType ctrl,
                                                 int pidSlot,
                                                 double arbFeedforward,
                                                 int arbFFUnits) {
    auto status = c_SparkMax_SetpointCommand(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), value,
        static_cast<c_SparkMax_ControlType>(ctrl), pidSlot, arbFeedforward,
        arbFFUnits);
    return static_cast<REVLibError>(status);
}

float CANSparkMaxLowLevel::GetSafeFloat(float f) {
    if (std::isinf(f) || std::isnan(f)) return 0;
    return f;
}

REVLibError CANSparkMaxLowLevel::RestoreFactoryDefaults(bool persist) {
    auto status = c_SparkMax_RestoreFactoryDefaults(
        static_cast<c_SparkMax_handle>(m_sparkMaxHandle), persist ? 1 : 0);
    return static_cast<REVLibError>(status);
}

void CANSparkMaxLowLevel::EnableExternalUSBControl(bool enable) {
    c_SparkMax_EnableExternalControl(enable);
}

#ifndef __FRC_ROBORIO__
void CANSparkMaxLowLevel::SetEnable(bool enable) {
    c_SparkMax_SetEnabled(enable);
}
#endif

}  // namespace rev
