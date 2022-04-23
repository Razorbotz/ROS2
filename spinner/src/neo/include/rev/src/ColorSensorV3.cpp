/*
 * Copyright (c) 2020-2021 REV Robotics
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

#include "rev/ColorSensorV3.h"

#include <algorithm>

#include <frc/Errors.h>
#include <frc/util/Color.h>

using namespace rev;

static constexpr int kAddress = 0x52;
static constexpr int kExpectedPartID = 0xC2;
static constexpr ColorSensorV3::GainFactor kDefaultGain =
    ColorSensorV3::GainFactor::k3x;
static constexpr ColorSensorV3::LEDPulseFrequency kDefaultPulseFreq =
    ColorSensorV3::LEDPulseFrequency::k60kHz;
static constexpr ColorSensorV3::LEDCurrent kDefaultLEDCurrent =
    ColorSensorV3::LEDCurrent::kPulse100mA;
static constexpr uint8_t kDefaultPulses = 32;
static constexpr ColorSensorV3::ProximityResolution kDefaultProxRes =
    ColorSensorV3::ProximityResolution::k11bit;
static constexpr ColorSensorV3::ProximityMeasurementRate kDefaultProxRate =
    ColorSensorV3::ProximityMeasurementRate::k50ms;
static constexpr ColorSensorV3::ColorResolution kDefaultColorRes =
    ColorSensorV3::ColorResolution::k18bit;
static constexpr ColorSensorV3::ColorMeasurementRate kDefaultColorRate =
    ColorSensorV3::ColorMeasurementRate::k100ms;

// This is a transformation matrix given by the chip
// manufacturer to transform the raw RGB to CIE XYZ
constexpr double ColorSensorV3::Cmatrix[9] = {
    0.048112847,  0.289453437,  -0.084950826, -0.030754752, 0.339680186,
    -0.071569905, -0.093947499, 0.072838494,  0.34024948};

ColorSensorV3::ColorSensorV3(frc::I2C::Port port)
    : m_i2c(port, kAddress),
      m_simDevice("REV Color Sensor V3", static_cast<int>(port), kAddress) {
    if (m_simDevice) {
        m_simR = m_simDevice.CreateDouble("Red", false, 0.0);
        m_simG = m_simDevice.CreateDouble("Green", false, 0.0);
        m_simB = m_simDevice.CreateDouble("Blue", false, 0.0);
        m_simIR = m_simDevice.CreateDouble("IR", false, 0.0);
        m_simProx = m_simDevice.CreateDouble("Proximity", false, 0.0);
        return;
    }

    // Only report errors if this is not a simulation
    if (!CheckDeviceID(!m_simDevice)) return;

    InitializeDevice();

    // Clear the reset flag
    HasReset();
}

uint32_t ColorSensorV3::GetProximity() {
    if (m_simProx) {
        return m_simProx.Get();
    }
    return Read11BitRegister(Register::kProximityData);
}

frc::Color ColorSensorV3::GetColor() {
    RawColor color = GetRawColor();
    double r = static_cast<double>(color.red);
    double g = static_cast<double>(color.green);
    double b = static_cast<double>(color.blue);
    double mag = r + g + b;
    return frc::Color(r / mag, g / mag, b / mag);

    // Possible future work: return sRGB values
    // return GetCIEColor().ToRGB();
}

ColorSensorV3::RawColor ColorSensorV3::GetRawColor() {
    uint8_t raw[12];

    if (m_simDevice) {
        return ColorSensorV3::RawColor(static_cast<uint32_t>(m_simR.Get()),
                                       static_cast<uint32_t>(m_simG.Get()),
                                       static_cast<uint32_t>(m_simB.Get()),
                                       static_cast<uint32_t>(m_simIR.Get()));
    }

    if (!Read(Register::kDataInfrared, 12, raw)) {
        return ColorSensorV3::RawColor(To20Bit(&raw[9]), To20Bit(&raw[3]),
                                       To20Bit(&raw[6]), To20Bit(&raw[0]));
    }

    return ColorSensorV3::RawColor(0, 0, 0, 0);
}

rev::CIEColor ColorSensorV3::GetCIEColor() {
    RawColor raw = GetRawColor();
    return rev::CIEColor(
        Cmatrix[0] * raw.red + Cmatrix[1] * raw.green + Cmatrix[2] * raw.blue,
        Cmatrix[3] * raw.red + Cmatrix[4] * raw.green + Cmatrix[5] * raw.blue,
        Cmatrix[6] * raw.red + Cmatrix[7] * raw.green + Cmatrix[8] * raw.blue);
}

double ColorSensorV3::GetIR() {
    if (m_simDevice) {
        return m_simIR.Get();
    }
    return static_cast<double>(Read20BitRegister(Register::kDataInfrared));
}

void ColorSensorV3::SetGain(ColorSensorV3::GainFactor gain) {
    Write(Register::kLightSensorGain, static_cast<uint8_t>(gain));
}

bool ColorSensorV3::CheckDeviceID(bool reportErrors) {
    uint8_t partID = 0;
    if (Read(Register::kPartID, 1, &partID)) {
        if (reportErrors) {
            FRC_ReportError(frc::err::Error, "{}",
                            "Could not find REV color sensor");
        }
        return false;
    }

    if (partID != kExpectedPartID) {
        if (reportErrors) {
            FRC_ReportError(
                frc::err::Error, "{}",
                "Unknown device found with same I2C address as REV color "
                "sensor");
        }
        return false;
    }

    return true;
}

ColorSensorV3::MainStatus ColorSensorV3::GetStatus() {
    MainStatus stat = {};
    Read(Register::kMainStatus, 1, reinterpret_cast<uint8_t*>(&stat));
    return stat;
}

void ColorSensorV3::InitializeDevice() {
    Write(Register::kMainCtrl,
          static_cast<uint8_t>(MainCtrlFields::kRGBMode) |
              static_cast<uint8_t>(MainCtrlFields::kLightSensorEnable) |
              static_cast<uint8_t>(MainCtrlFields::kProximitySensorEnable));

    ConfigureProximitySensorLED(kDefaultPulseFreq, kDefaultLEDCurrent,
                                kDefaultPulses);
    ConfigureProximitySensor(kDefaultProxRes, kDefaultProxRate);
    ConfigureColorSensor(kDefaultColorRes, kDefaultColorRate);
    SetGain(kDefaultGain);
}

void ColorSensorV3::ConfigureProximitySensorLED(LEDPulseFrequency freq,
                                                LEDCurrent curr,
                                                uint8_t pulses) {
    Write(Register::kProximitySensorLED,
          static_cast<uint8_t>(freq) | static_cast<uint8_t>(curr));

    Write(Register::kProximitySensorPulses, pulses);
}

void ColorSensorV3::ConfigureProximitySensor(ProximityResolution res,
                                             ProximityMeasurementRate rate) {
    Write(Register::kProximitySensorRate,
          static_cast<uint8_t>(res) | static_cast<uint8_t>(rate));
}

void ColorSensorV3::ConfigureColorSensor(ColorResolution res,
                                         ColorMeasurementRate rate) {
    Write(Register::kLightSensorMeasurementRate,
          static_cast<uint8_t>(res) | static_cast<uint8_t>(rate));
}

bool ColorSensorV3::HasReset() {
    if (m_simDevice) {
        return false;
    }
    return GetStatus().PowerOnStatus != 0;
}

bool ColorSensorV3::IsConnected() {
    if (m_simDevice) {
        return true;
    }

    return CheckDeviceID(false);
}

uint16_t ColorSensorV3::Read11BitRegister(Register reg) {
    uint8_t raw[2];

    m_i2c.Read(static_cast<uint8_t>(reg), 2, raw);

    return To11Bit(raw);
}

uint32_t ColorSensorV3::Read20BitRegister(Register reg) {
    uint8_t raw[3];

    m_i2c.Read(static_cast<uint8_t>(reg), 3, raw);

    return To20Bit(raw);
}
