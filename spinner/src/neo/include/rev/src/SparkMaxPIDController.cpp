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

#include "rev/SparkMaxPIDController.h"

#include "rev/CANSparkMax.h"
#include "rev/CANSparkMaxDriver.h"

using namespace rev;

SparkMaxPIDController::SparkMaxPIDController(CANSparkMax& device)
    : m_device(&device) {}

REVLibError SparkMaxPIDController::SetReference(
    double value, CANSparkMaxLowLevel::ControlType ctrl, int pidSlot,
    double arbFF, SparkMaxPIDController::ArbFFUnits arbFFUnits) {
    auto status = c_SparkMax_SetpointCommand(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), value,
        static_cast<c_SparkMax_ControlType>(ctrl), pidSlot, arbFF,
        static_cast<int>(arbFFUnits));
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxPIDController::SetReference(
    double value, ControlType ctrl, int pidSlot, double arbFF,
    CANPIDController::ArbFFUnits arbFFUnits) {
    auto status = c_SparkMax_SetpointCommand(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), value,
        static_cast<c_SparkMax_ControlType>(ctrl), pidSlot, arbFF,
        static_cast<int>(arbFFUnits));
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxPIDController::SetP(double gain, int slotID) {
    auto status = c_SparkMax_SetP(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        gain);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxPIDController::SetI(double gain, int slotID) {
    auto status = c_SparkMax_SetI(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        gain);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxPIDController::SetD(double gain, int slotID) {
    auto status = c_SparkMax_SetD(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        gain);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxPIDController::SetDFilter(double gain, int slotID) {
    auto status = c_SparkMax_SetDFilter(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        gain);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxPIDController::SetFF(double gain, int slotID) {
    auto status = c_SparkMax_SetFF(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        gain);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxPIDController::SetIZone(double IZone, int slotID) {
    auto status = c_SparkMax_SetIZone(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        IZone);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxPIDController::SetOutputRange(double min, double max,
                                                  int slotID) {
    auto status = c_SparkMax_SetOutputRange(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID, min,
        max);
    return static_cast<REVLibError>(status);
}

double SparkMaxPIDController::GetP(int slotID) const {
    float tmp;
    c_SparkMax_GetP(static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle),
                    slotID, &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxPIDController::GetI(int slotID) const {
    float tmp;
    c_SparkMax_GetI(static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle),
                    slotID, &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxPIDController::GetD(int slotID) const {
    float tmp;
    c_SparkMax_GetD(static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle),
                    slotID, &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxPIDController::GetDFilter(int slotID) const {
    float tmp;
    c_SparkMax_GetDFilter(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxPIDController::GetFF(int slotID) const {
    float tmp;
    c_SparkMax_GetFF(static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle),
                     slotID, &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxPIDController::GetIZone(int slotID) const {
    float tmp;
    c_SparkMax_GetIZone(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxPIDController::GetOutputMin(int slotID) const {
    float tmp;
    c_SparkMax_GetOutputMin(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxPIDController::GetOutputMax(int slotID) const {
    float tmp;
    c_SparkMax_GetOutputMax(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        &tmp);
    return static_cast<double>(tmp);
}

REVLibError SparkMaxPIDController::SetSmartMotionMaxVelocity(double maxVel,
                                                             int slotID) {
    auto status = c_SparkMax_SetSmartMotionMaxVelocity(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        maxVel);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxPIDController::SetSmartMotionMaxAccel(double maxAccel,
                                                          int slotID) {
    auto status = c_SparkMax_SetSmartMotionMaxAccel(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        maxAccel);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxPIDController::SetSmartMotionMinOutputVelocity(
    double minVel, int slotID) {
    auto status = c_SparkMax_SetSmartMotionMinOutputVelocity(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        minVel);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxPIDController::SetSmartMotionAllowedClosedLoopError(
    double allowedErr, int slotID) {
    auto status = c_SparkMax_SetSmartMotionAllowedClosedLoopError(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        allowedErr);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxPIDController::SetSmartMotionAccelStrategy(
    SparkMaxPIDController::AccelStrategy accelStrategy, int slotID) {
    auto status = c_SparkMax_SetSmartMotionAllowedClosedLoopError(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        static_cast<c_SparkMax_AccelStrategy>(accelStrategy));
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxPIDController::SetSmartMotionAccelStrategy(
    CANPIDController::AccelStrategy accelStrategy, int slotID) {
    auto status = c_SparkMax_SetSmartMotionAllowedClosedLoopError(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        static_cast<c_SparkMax_AccelStrategy>(accelStrategy));
    return static_cast<REVLibError>(status);
}

double SparkMaxPIDController::GetSmartMotionMaxVelocity(int slotID) const {
    float tmp;
    c_SparkMax_GetSmartMotionMaxVelocity(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxPIDController::GetSmartMotionMaxAccel(int slotID) const {
    float tmp;
    c_SparkMax_GetSmartMotionMaxAccel(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxPIDController::GetSmartMotionMinOutputVelocity(
    int slotID) const {
    float tmp;
    c_SparkMax_GetSmartMotionMinOutputVelocity(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxPIDController::GetSmartMotionAllowedClosedLoopError(
    int slotID) const {
    float tmp;
    c_SparkMax_GetSmartMotionAllowedClosedLoopError(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        &tmp);
    return static_cast<double>(tmp);
}

SparkMaxPIDController::AccelStrategy
SparkMaxPIDController::GetSmartMotionAccelStrategy(int slotID) const {
    c_SparkMax_AccelStrategy tmp;
    c_SparkMax_GetSmartMotionAccelStrategy(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        &tmp);
    return static_cast<SparkMaxPIDController::AccelStrategy>(tmp);
}

REVLibError SparkMaxPIDController::SetIMaxAccum(double iMaxAccum, int slotID) {
    auto status = c_SparkMax_SetIMaxAccum(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        iMaxAccum);
    return static_cast<REVLibError>(status);
}

double SparkMaxPIDController::GetIMaxAccum(int slotID) const {
    float tmp;
    c_SparkMax_GetIMaxAccum(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), slotID,
        &tmp);
    return static_cast<double>(tmp);
}

REVLibError SparkMaxPIDController::SetIAccum(double iAccum) {
    auto status = c_SparkMax_SetIAccum(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), iAccum);
    return static_cast<REVLibError>(status);
}

double SparkMaxPIDController::GetIAccum() const {
    float tmp;
    c_SparkMax_GetIAccum(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

REVLibError SparkMaxPIDController::SetFeedbackDevice(const CANSensor& sensor) {
    auto status = c_SparkMax_SetFeedbackDevice(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle),
        sensor.GetSparkMaxFeedbackDeviceID());
    return static_cast<REVLibError>(status);
}
