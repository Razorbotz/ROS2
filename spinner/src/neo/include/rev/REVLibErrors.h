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

#pragma once

#include <string>

extern "C" {

#if 0
#ifdef __GNUC__
#define __FORMAT_TEXT(A, B) __attribute__((__format__(__printf__, (A), (B))))
#else
#define __FORMAT_TEXT(A, B)
#endif
#endif

typedef enum _c_REVLib_ErrorCode {
    c_REVLibError_None = 0,
    c_REVLibError_General,
    c_REVLibError_CANTimeout,
    c_REVLibError_NotImplemented,
    c_REVLibError_HAL,
    c_REVLibError_CantFindFirmware,
    c_REVLibError_FirmwareTooOld,
    c_REVLibError_FirmwareTooNew,
    c_REVLibError_ParamInvalidID,
    c_REVLibError_ParamMismatchType,
    c_REVLibError_ParamAccessMode,
    c_REVLibError_ParamInvalid,
    c_REVLibError_ParamNotImplementedDeprecated,
    c_REVLibError_FollowConfigMismatch,
    c_REVLibError_Invalid,
    c_REVLibError_SetpointOutOfRange,
    c_REVLibError_Unknown,
    c_REVLibError_CANDisconnected,
    c_REVLibError_DuplicateCANId,
    c_REVLibError_InvalidCANId,
    c_REVLibError_SparkMaxDataPortAlreadyConfiguredDifferently,
    c_REVLibError_NumCodes,
} c_REVLib_ErrorCode;

void c_REVLib_SendError(c_REVLib_ErrorCode code, int deviceId);
void c_REVLib_SendErrorText(c_REVLib_ErrorCode code, int deviceId,
                            std::string context);

void c_REVLib_FlushErrors(void);
void c_REVLib_SuppressErrors(bool suppress);
int c_REVLib_ErrorSize(void);
const char* c_REVLib_ErrorFromCode(c_REVLib_ErrorCode code);

}  // extern "C"
