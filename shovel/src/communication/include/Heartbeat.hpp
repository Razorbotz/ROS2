#pragma once
#include <cstdint>

/**
 * @file heartbeat_protocol.hpp
 *
 * @brief Definition of the hybrid Heartbeat/Data protocol.
 *
 * @section Protocol_Structure
 * All packets verify liveness. The protocol uses a common header format
 * to ensure that Heartbeats and Data packets can be handled by the same
 * receiving loop.
 *
 * Memory Layout:
 * [ Header (16 bytes) ]
 * |
 * +--> [ Optional: Data ID (2 bytes) ]
 * +--> [ Optional: Length  (2 bytes) ]
 * +--> [ Optional: Payload (N bytes) ]
 */

static constexpr uint16_t HB_MAGIC = 0xBEEF;
static constexpr uint8_t  HB_VER   = 1;
static constexpr uint32_t HB_INTERVAL_MS = 50;
static constexpr uint32_t HB_TIMEOUT_MS  = 500;

// --- MESSAGE TYPES ---
static constexpr uint8_t MSG_TYPE_HEARTBEAT = 0x01;
static constexpr uint8_t MSG_TYPE_DATA      = 0x02;

#pragma pack(push, 1)

/**
 * @brief Common Header.
 *
 * Every packet sent over this protocol MUST start with this structure.
 * This allows the receiver to peek at the `type` field before deciding
 * how much more data to read.
 */
struct NanoHeader {
    /**
     * @brief Integrity Check.
     * Must match HB_MAGIC. Used to reject garbage traffic on the port.
     */
    uint16_t magic;

    /**
     * @brief Discriminator.
     * 0x01 = Heartbeat (Stop reading here).
     * 0x02 = Data (Read NanoDataPacket fields).
     */
    uint8_t  type;

    /**
     * @brief Compatibility version.
     * Packets with mismatched versions should be discarded.
     */
    uint8_t  version;

    /**
     * @brief Packet Sequence Number.
     * Monotonically increasing. Used to detect packet loss (gaps in sequence)
     * or out-of-order delivery.
     */
    uint32_t seq;

    /**
     * @brief Sender Timestamp (ms).
     * Used to calculate one-way latency (receiver_time - t_ms).
     */
    uint64_t t_ms;
};

/**
 * @brief Data Packet.
 * Extends the header to include a variable length payload.
 * Motor IDs: 
 * 10 - 13 : TalonFX motor controllers
 * 14 - 16 : TalonSRX motor controllers
 * 100 : Message to use the Nano to control motors
 * 101 : Confirmation of motor control from Nano to Orin
 * 200 : Query from Orin to Nano if it is in control
 * 201 : Response from Nano to Orin that it is in control
 * 202 : Response from Nano to Orin that it is not in control
 * 203 : Request from Orin to Nano to retake control
 * 204 : Response from Nano to Orin to take control
 * 205 : Response from Nano to Orin to not take control
 *     - This will include a message for how many seconds to delay
 */
struct NanoDataPacket {
    NanoHeader header;
    uint16_t   data_id;      // specific ID for the data
    uint16_t   payload_len;  // Length of the following data
    uint8_t    payload[1024];// Max payload buffer
};

#pragma pack(pop)
