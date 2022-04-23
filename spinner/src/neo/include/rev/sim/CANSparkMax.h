/*
 * Copyright (c) 2020-2022 REV Robotics
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

#pragma once

#include <stdint.h>

#include "rev/REVLibErrors.h"

extern "C" {

typedef struct c_SIM_SparkMax_Obj* c_SIM_SparkMax_handle;

c_SIM_SparkMax_handle c_SIM_SparkMax_Create(int deviceId);
void c_SIM_SparkMax_Destory(c_SIM_SparkMax_handle handle);
bool c_SIM_SparkMax_IsSim(c_SIM_SparkMax_handle handle);

c_REVLib_ErrorCode c_SIM_SparkMax_SetParameter(c_SIM_SparkMax_handle handle,
                                               uint8_t parameterID,
                                               uint8_t type, uint32_t value);
c_REVLib_ErrorCode c_SIM_SparkMax_GetParameter(c_SIM_SparkMax_handle handle,
                                               uint8_t parameterID,
                                               uint8_t type, uint32_t* value);

// New sim functions, telemetry 'get' functions only, HAL_SimDevice Values
// handle setting
uint32_t c_SIM_SparkMax_GetFirmwareVersion(c_SIM_SparkMax_handle handle);
float c_SIM_SparkMax_GetAppliedOutput(c_SIM_SparkMax_handle handle);
uint16_t c_SIM_SparkMax_GetFaults(c_SIM_SparkMax_handle handle);
uint8_t c_SIM_SparkMax_GetInverted(c_SIM_SparkMax_handle handle);
uint8_t c_SIM_SparkMax_GetIsFollower(c_SIM_SparkMax_handle handle);
uint16_t c_SIM_SparkMax_GetStickyFaults(c_SIM_SparkMax_handle handle);
float c_SIM_SparkMax_GetOutputCurrent(c_SIM_SparkMax_handle handle);
float c_SIM_SparkMax_GetBusVoltage(c_SIM_SparkMax_handle handle);
uint8_t c_SIM_SparkMax_GetMotorTemperature(c_SIM_SparkMax_handle handle);
float c_SIM_SparkMax_GetVelocity(c_SIM_SparkMax_handle handle);
float c_SIM_SparkMax_GetPosition(c_SIM_SparkMax_handle handle);
float c_SIM_SparkMax_GetAnalogVoltage(c_SIM_SparkMax_handle handle);
float c_SIM_SparkMax_GetAnalogPosition(c_SIM_SparkMax_handle handle);
float c_SIM_SparkMax_GetAnalogVelocity(c_SIM_SparkMax_handle handle);
float c_SIM_SparkMax_GetAltEncoderPosition(c_SIM_SparkMax_handle handle);
float c_SIM_SparkMax_GetAltEncoderVelocity(c_SIM_SparkMax_handle handle);
void c_SIM_SparkMax_SetAppliedOutput(c_SIM_SparkMax_handle handle, float value);
void c_SIM_SparkMax_SetAnalogVoltage(c_SIM_SparkMax_handle handle, float value);
void c_SIM_SparkMax_SetAnalogPosition(c_SIM_SparkMax_handle handle,
                                      float value);
void c_SIM_SparkMax_SetAnalogVelocity(c_SIM_SparkMax_handle handle,
                                      float value);
void c_SIM_SparkMax_SetAltEncoderVelocity(c_SIM_SparkMax_handle handle,
                                          float value);

c_REVLib_ErrorCode c_SIM_SparkMax_SetFollow(c_SIM_SparkMax_handle handle,
                                            uint32_t followerArbId,
                                            uint32_t followerCfg);
c_REVLib_ErrorCode c_SIM_SparkMax_SetTelemetry(c_SIM_SparkMax_handle handle,
                                               float value, int apiId);
c_REVLib_ErrorCode c_SIM_SparkMax_RestoreFactoryDefaults(
    c_SIM_SparkMax_handle handle, bool persist, bool resetAll);
c_REVLib_ErrorCode c_SIM_SparkMax_SetSetpoint(c_SIM_SparkMax_handle handle,
                                              float value, uint8_t ctrl,
                                              int pidSlot, float arbFeedforward,
                                              int arbFFUnits);
c_REVLib_ErrorCode c_SIM_SparkMax_GetDRVStatus(c_SIM_SparkMax_handle handle,
                                               uint16_t* DRVStat0,
                                               uint16_t* DRVStat1,
                                               uint16_t* faults,
                                               uint16_t* stickyFaults);
c_REVLib_ErrorCode c_SIM_SparkMax_ClearFaults(c_SIM_SparkMax_handle handle);
c_REVLib_ErrorCode c_SIM_SparkMax_SetFreeSpeed(c_SIM_SparkMax_handle handle,
                                               float freeSpeed);
c_REVLib_ErrorCode c_SIM_SparkMax_SetStallTorque(c_SIM_SparkMax_handle handle,
                                                 float stallTorque);
c_REVLib_ErrorCode c_SIM_SparkMax_GetPositionConversionFactor(
    c_SIM_SparkMax_handle handle, float* factor);
c_REVLib_ErrorCode c_SIM_SparkMax_GetVelocityConversionFactor(
    c_SIM_SparkMax_handle handle, float* factor);
c_REVLib_ErrorCode c_SIM_SparkMax_SetPositionConversionFactor(
    c_SIM_SparkMax_handle handle, float factor);
c_REVLib_ErrorCode c_SIM_SparkMax_SetVelocityConversionFactor(
    c_SIM_SparkMax_handle handle, float factor);

}  // extern "C"
