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

#include "rev/CANDeviceScanner.h"

#include <stdint.h>

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

#include <hal/CAN.h>
#include <hal/HALBase.h>

#include "rev/CANSparkMaxDriver.h"
#include "rev/CANSparkMaxFrames.h"

using namespace rev;
using namespace rev::detail;

#define CAN_DEVICEID_FIELD_MASK 0x3F

CANBusScanner::CANBusScanner(int bufferSize, int threadInterval)
    : m_streamBufferSize(bufferSize),
      m_streamHandle(0),
      m_threadInterval(0),
      m_thread(&CANBusScanner::run, this),
      m_running(false) {}

CANBusScanner::~CANBusScanner() {
    Stop();
    m_stopThread = true;
    m_thread.join();
}

bool CANBusScanner::Start() {
    int32_t status = 0;
    HAL_CAN_OpenStreamSession(&m_streamHandle, 0, 0, m_streamBufferSize,
                              &status);
    if (status != 0) {
        if (m_streamHandle != 0) {
            HAL_CAN_CloseStreamSession(m_streamHandle);
        }
        m_lastError =
            "Unable to open stream session, status: " + std::to_string(status);
        m_streamHandle = 0;
        return false;
    }
    m_lastError = std::string();
    m_running = true;
    return true;
}

void CANBusScanner::Stop() {
    if (m_streamHandle != 0) {
        HAL_CAN_CloseStreamSession(m_streamHandle);
    }
    m_streamHandle = 0;
    m_running = false;
}

bool CANBusScanner::Running() { return m_running; }

std::string CANBusScanner::LastError() { return m_lastError; }

void CANBusScanner::run() {
    HAL_CANStreamMessage* msgbuf = new HAL_CANStreamMessage[m_streamBufferSize];
    while (!m_stopThread) {
        if (m_running) {
            uint32_t numRead = 0;
            int32_t status = 0;
            HAL_CAN_ReadStreamSession(m_streamHandle, msgbuf,
                                      m_streamBufferSize, &numRead, &status);

            if (status != 0) {
                Stop();
                m_lastError = "Failed to read stream session, status:" +
                              std::to_string(status);
            } else {
                for (int i = 0; i < static_cast<int>(numRead); i++) {
                    // Read entire buffer
                    auto element = m_registeredDevices.find(
                        msgbuf[i].messageID & ~CAN_DEVICEID_FIELD_MASK);
                    if (element != m_registeredDevices.end()) {
                        // Message is registered
                        int canID =
                            msgbuf[i].messageID & CAN_DEVICEID_FIELD_MASK;

                        element->second->AddOrUpdateDevice(canID);
                    }
                }
            }
        }

        if (!m_stopThread) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(m_threadInterval));
        }
    }
    delete[] msgbuf;
}

void CANBusScanner::RegisterDevice(std::string name,
                                   std::vector<uint32_t> validIds,
                                   int32_t maxFramePeriodMs) {
    if (validIds.empty()) {
        return;
    }

    std::shared_ptr<CANScanCollection> ptr =
        std::make_shared<CANScanCollection>(name, validIds[0],
                                            maxFramePeriodMs);
    m_registeredList.push_back(ptr);
    for (auto const& id : validIds) {
        m_registeredDevices[id] = ptr;
    }
}

std::vector<CANScanIdentifier> CANBusScanner::CANBusScan() {
    std::vector<CANScanIdentifier> result;
    for (auto const& devElements : m_registeredList) {
        uint32_t arbid = devElements->ArbId();
        std::vector<int> deviceIds = devElements->ActiveDevices();

        for (auto const& devId : deviceIds) {
            result.push_back(
                CANScanIdentifier(arbid | devId, devElements->Name()));
        }
    }
    return result;
}

CANBusScanner::CANScanElement::CANScanElement(uint64_t timeoutMs) {
    this->lastSeen = 0;

    // Internally stored as micro-senconds
    this->timeout = timeoutMs * 1000;
}

void CANBusScanner::CANScanElement::UpdateLastSeen() {
    int32_t status = 0;
    this->lastSeen = HAL_GetFPGATime(&status);
}

bool CANBusScanner::CANScanElement::IsActive() const {
    int32_t status = 0;
    uint64_t timenow = HAL_GetFPGATime(&status);

    if (timenow > (this->lastSeen + this->timeout)) {
        return false;
    }

    return true;
}
