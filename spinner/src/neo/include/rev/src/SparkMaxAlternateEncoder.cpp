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

#include "rev/SparkMaxAlternateEncoder.h"

#include <stdexcept>

#include <fmt/format.h>

#include "rev/CANSparkMax.h"
#include "rev/CANSparkMaxDriver.h"

using namespace rev;

SparkMaxAlternateEncoder::SparkMaxAlternateEncoder(CANSparkMax& device,
                                                   Type type, int countsPerRev)
    : RelativeEncoder(), m_device(&device), m_countsPerRev(countsPerRev) {
    if (m_countsPerRev < 1) {
        throw std::invalid_argument("countsPerRev must be a positive number");
    }

    if (c_SparkMax_AttemptToSetDataPortConfig(
            static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle),
            c_SparkMax_kDataPortConfigAltEncoder) ==
        c_REVLibError_SparkMaxDataPortAlreadyConfiguredDifferently) {
        throw std::runtime_error(fmt::format(
            "An alternate encoder cannot be used on SPARK MAX #{}, because it "
            "has a limit switch configured",
            m_device->m_deviceID));
    }

    c_SparkMax_SetAltEncoderCountsPerRevolution(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle),
        countsPerRev);
}

double SparkMaxAlternateEncoder::GetPosition() const {
    float tmp;

    c_SparkMax_GetAltEncoderPosition(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxAlternateEncoder::GetVelocity() const {
    float tmp;
    c_SparkMax_GetAltEncoderVelocity(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

REVLibError SparkMaxAlternateEncoder::SetPosition(double position) {
    c_REVLib_ErrorCode status;
    status = c_SparkMax_SetAltEncoderPosition(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), position);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxAlternateEncoder::SetPositionConversionFactor(
    double factor) {
    c_REVLib_ErrorCode status;
    status = c_SparkMax_SetAltEncoderPositionFactor(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), factor);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxAlternateEncoder::SetVelocityConversionFactor(
    double factor) {
    c_REVLib_ErrorCode status;
    status = c_SparkMax_SetAltEncoderVelocityFactor(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), factor);
    return static_cast<REVLibError>(status);
}

double SparkMaxAlternateEncoder::GetPositionConversionFactor() const {
    float tmp;
    c_SparkMax_GetAltEncoderPositionFactor(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxAlternateEncoder::GetVelocityConversionFactor() const {
    float tmp;
    c_SparkMax_GetAltEncoderVelocityFactor(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

REVLibError SparkMaxAlternateEncoder::SetAverageDepth(uint32_t depth) {
    c_REVLib_ErrorCode status;
    status = c_SparkMax_SetAltEncoderAverageDepth(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), depth);
    return static_cast<REVLibError>(status);
}

uint32_t SparkMaxAlternateEncoder::GetAverageDepth() const {
    uint32_t tmp;
    c_SparkMax_GetAltEncoderAverageDepth(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<uint32_t>(tmp);
}

REVLibError SparkMaxAlternateEncoder::SetMeasurementPeriod(uint32_t samples) {
    c_REVLib_ErrorCode status;
    status = c_SparkMax_SetAltEncoderMeasurementPeriod(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), samples);
    return static_cast<REVLibError>(status);
}

uint32_t SparkMaxAlternateEncoder::GetMeasurementPeriod() const {
    uint32_t tmp;
    c_SparkMax_GetAltEncoderMeasurementPeriod(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<uint32_t>(tmp);
}

uint32_t SparkMaxAlternateEncoder::GetCountsPerRevolution() const {
    uint32_t tmp;
    c_SparkMax_GetAltEncoderCountsPerRevolution(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<uint32_t>(tmp);
}

int SparkMaxAlternateEncoder::GetSparkMaxFeedbackDeviceID() const {
    return static_cast<int>(
        CANSparkMaxLowLevel::FeedbackSensorType::kAltQuadrature);
}

REVLibError SparkMaxAlternateEncoder::SetInverted(bool inverted) {
    auto status = c_SparkMax_SetAltEncoderInverted(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), inverted);
    return static_cast<REVLibError>(status);
}

bool SparkMaxAlternateEncoder::GetInverted() const {
    uint8_t inverted;
    c_SparkMax_GetAltEncoderInverted(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &inverted);
    return inverted ? true : false;
}
