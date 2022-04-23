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

#include "rev/SparkMaxAnalogSensor.h"

#include "rev/CANSparkMax.h"
#include "rev/CANSparkMaxDriver.h"

using namespace rev;

SparkMaxAnalogSensor::SparkMaxAnalogSensor(CANSparkMax& device, Mode mode)
    : AnalogInput(), CANAnalog(), m_device(&device), m_mode(mode) {
    c_SparkMax_SetAnalogMode(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle),
        static_cast<c_SparkMax_AnalogMode>(mode));
}

double SparkMaxAnalogSensor::GetPosition() const {
    float tmp;
    c_SparkMax_GetAnalogPosition(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxAnalogSensor::GetVelocity() const {
    float tmp;
    c_SparkMax_GetAnalogVelocity(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxAnalogSensor::GetVoltage() const {
    float tmp;
    c_SparkMax_GetAnalogVoltage(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

void SparkMaxAnalogSensor::SetSimPosition(double position) {
    c_SparkMax_SetSimAnalogPosition(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), position);
}

void SparkMaxAnalogSensor::SetSimVelocity(double velocity) {
    c_SparkMax_SetSimAnalogVelocity(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), velocity);
}

void SparkMaxAnalogSensor::SetSimVoltage(double voltage) {
    c_SparkMax_SetSimAnalogVoltage(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), voltage);
}

REVLibError SparkMaxAnalogSensor::SetPositionConversionFactor(double factor) {
    auto status = c_SparkMax_SetAnalogPositionConversionFactor(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), factor);
    return static_cast<REVLibError>(status);
}

double SparkMaxAnalogSensor::GetPositionConversionFactor() const {
    float tmp;
    c_SparkMax_GetAnalogPositionConversionFactor(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

REVLibError SparkMaxAnalogSensor::SetVelocityConversionFactor(double factor) {
    auto status = c_SparkMax_SetAnalogVelocityConversionFactor(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), factor);
    return static_cast<REVLibError>(status);
}

double SparkMaxAnalogSensor::GetVelocityConversionFactor() const {
    float tmp;
    c_SparkMax_GetAnalogVelocityConversionFactor(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

int SparkMaxAnalogSensor::GetSparkMaxFeedbackDeviceID() const {
    return static_cast<uint32_t>(
        CANSparkMaxLowLevel::FeedbackSensorType::kAnalog);
}

REVLibError SparkMaxAnalogSensor::SetInverted(bool inverted) {
    auto status = c_SparkMax_SetAnalogInverted(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), inverted);
    return static_cast<REVLibError>(status);
}

bool SparkMaxAnalogSensor::GetInverted() const {
    uint8_t inverted;
    c_SparkMax_GetAnalogInverted(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &inverted);
    return inverted ? true : false;
}

REVLibError SparkMaxAnalogSensor::SetAverageDepth(uint32_t depth) {
    auto status = c_SparkMax_SetAnalogAverageDepth(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), depth);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxAnalogSensor::SetMeasurementPeriod(uint32_t period_ms) {
    auto status = c_SparkMax_SetAnalogMeasurementPeriod(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), period_ms);
    return static_cast<REVLibError>(status);
}

uint32_t SparkMaxAnalogSensor::GetAverageDepth() const {
    uint32_t depth = 0;
    c_SparkMax_GetAnalogAverageDepth(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &depth);
    return depth;
}

uint32_t SparkMaxAnalogSensor::GetMeasurementPeriod() const {
    uint32_t period_ms = 0;
    c_SparkMax_GetAnalogMeasurementPeriod(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &period_ms);
    return period_ms;
}
