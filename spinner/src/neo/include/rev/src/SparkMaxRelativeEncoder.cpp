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

#include "rev/SparkMaxRelativeEncoder.h"

#include <stdexcept>

#include "rev/CANSparkMax.h"
#include "rev/CANSparkMaxDriver.h"

using namespace rev;

SparkMaxRelativeEncoder::SparkMaxRelativeEncoder(CANSparkMax& device, Type type,
                                                 int countsPerRev)
    : RelativeEncoder(), m_device(&device), m_countsPerRev(countsPerRev) {
    bool setCountsPerRevParameter = true;

    if (type == Type::kHallSensor) {
        // The CPR for a hall sensor is hardcoded in the firmware as 42
        setCountsPerRevParameter = false;

        if (m_device->m_motorType == CANSparkMax::MotorType::kBrushed) {
            throw std::runtime_error(
                "A hall sensor cannot be used with a brushed motor");
        }

        // Make sure that the correct CPR value was provided
        if (m_countsPerRev != 42 && m_countsPerRev != 0) {
            throw std::invalid_argument(
                "countsPerRev must be 42 when using the hall sensor");
        }
    } else {  // non-hall sensor
        if (m_device->m_motorType == CANSparkMax::MotorType::kBrushless) {
            throw std::runtime_error(
                "The encoder type must be kHallSensor when the SPARK MAX is "
                "configured in brushless mode.\nTo use an external quadrature "
                "encoder with a brushless motor, you must wire it as an "
                "Alternate Encoder, and then call getAlternateEncoder() on the "
                "CANSparkMax object.");
        }

        if (m_countsPerRev < 1) {
            throw std::invalid_argument(
                "countsPerRev must be a positive number");
        }
    }

    // Unhandled REVLibError is returned by the c_SparkMax_SetSensorType and
    // c_SparkMax_SetCPR calls
    c_SparkMax_SetSensorType(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle),
        static_cast<c_SparkMax_EncoderType>(type));

    if (setCountsPerRevParameter) {
        c_SparkMax_SetCountsPerRevolution(
            static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle),
            m_countsPerRev);
    }
}

double SparkMaxRelativeEncoder::GetPosition() const {
    float tmp;

    c_SparkMax_GetEncoderPosition(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxRelativeEncoder::GetVelocity() const {
    float tmp;
    c_SparkMax_GetEncoderVelocity(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

REVLibError SparkMaxRelativeEncoder::SetPosition(double position) {
    c_REVLib_ErrorCode status;
    status = c_SparkMax_SetEncoderPosition(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), position);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxRelativeEncoder::SetPositionConversionFactor(
    double factor) {
    c_REVLib_ErrorCode status;
    status = c_SparkMax_SetPositionConversionFactor(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), factor);
    return static_cast<REVLibError>(status);
}

REVLibError SparkMaxRelativeEncoder::SetVelocityConversionFactor(
    double factor) {
    c_REVLib_ErrorCode status;
    status = c_SparkMax_SetVelocityConversionFactor(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), factor);
    return static_cast<REVLibError>(status);
}

double SparkMaxRelativeEncoder::GetPositionConversionFactor() const {
    float tmp;
    c_SparkMax_GetPositionConversionFactor(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

double SparkMaxRelativeEncoder::GetVelocityConversionFactor() const {
    float tmp;
    c_SparkMax_GetVelocityConversionFactor(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<double>(tmp);
}

REVLibError SparkMaxRelativeEncoder::SetAverageDepth(uint32_t depth) {
    c_REVLib_ErrorCode status;
    status = c_SparkMax_SetAverageDepth(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), depth);
    return static_cast<REVLibError>(status);
}

uint32_t SparkMaxRelativeEncoder::GetAverageDepth() const {
    uint32_t tmp;
    c_SparkMax_GetAverageDepth(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<uint32_t>(tmp);
}

REVLibError SparkMaxRelativeEncoder::SetMeasurementPeriod(uint32_t samples) {
    c_REVLib_ErrorCode status;
    status = c_SparkMax_SetMeasurementPeriod(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), samples);
    return static_cast<REVLibError>(status);
}

uint32_t SparkMaxRelativeEncoder::GetMeasurementPeriod() const {
    uint32_t tmp;
    c_SparkMax_GetMeasurementPeriod(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<uint32_t>(tmp);
}

uint32_t SparkMaxRelativeEncoder::GetCountsPerRevolution() const {
    uint32_t tmp;
    c_SparkMax_GetCountsPerRevolution(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &tmp);
    return static_cast<uint32_t>(tmp);
}

int SparkMaxRelativeEncoder::GetSparkMaxFeedbackDeviceID() const {
    return static_cast<int>(
        (static_cast<c_SparkMax_MotorType>(m_device->GetMotorType()) ==
         c_SparkMax_kBrushless)
            ? CANSparkMaxLowLevel::FeedbackSensorType::kHallSensor
            : CANSparkMaxLowLevel::FeedbackSensorType::kQuadrature);
}

REVLibError SparkMaxRelativeEncoder::SetInverted(bool inverted) {
    if (static_cast<c_SparkMax_MotorType>(m_device->GetInitialMotorType()) ==
        c_SparkMax_kBrushless) {
        throw std::invalid_argument("Not available in Brushless Mode");
    }

    auto status = c_SparkMax_SetEncoderInverted(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), inverted);
    return static_cast<REVLibError>(status);
}

bool SparkMaxRelativeEncoder::GetInverted() const {
    uint8_t inverted;
    c_SparkMax_GetEncoderInverted(
        static_cast<c_SparkMax_handle>(m_device->m_sparkMaxHandle), &inverted);
    return inverted ? true : false;
}
