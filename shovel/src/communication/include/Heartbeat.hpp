#pragma once
#include <cstdint>
#include <vector>
#include <atomic>
#include <netinet/in.h>
#include <functional>

// --- PROTOCOL CONSTANTS ---
static constexpr uint16_t HB_MAGIC = 0xBEEF;
static constexpr uint8_t  HB_VER   = 1;

// UPDATED: 10ms interval / 50ms timeout per your Resiliency Plan
static constexpr uint32_t HB_INTERVAL_MS = 10; 
static constexpr uint32_t HB_TIMEOUT_MS  = 50;

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
 * Data IDs: 
 * 001 : Motor speed values
 *     - This will have motor ID and speed value (float 32)
 * 002 : Motor position value
 *     - This will have motor ID and position (int32) 
 * 100 : Message to use the Nano to control motors
 *     - This will include a message with the motors to control
 *     - This will be a list of motor IDs to control
 * 101 : Confirmation of motor control from Nano to Orin
 * 200 : Query from Orin to Nano if it is in control
 * 201 : Response from Nano to Orin that it is in control
 * 202 : Response from Nano to Orin that it is not in control
 * 203 : Request from Orin to Nano to retake control
 * 204 : Response from Nano to Orin to take control
 * 205 : Response from Nano to Orin to not take control
 *     - This will include a message for how many seconds to delay
 * 206 : Message from Orin to Nano that it regained control
 *     - Include list of regained motor IDs
 * 207 : Message from Orin to Nano that it regained control
 *     - Include list of regained motor IDs
 * 400 : Message to force Nano to stop immediately
 * 401 : Message to have the Nano stop gracefully
 * 402 : Message from the Orin to the Nano that it lost control of motors
 *     - Include a list of lost motor IDs
 * 403 : Message from Nano to Orin that it lost motors
 *     - Include a list of lost motor IDs
 * 
 */
struct NanoDataPacket {
    NanoHeader header;
    uint16_t   data_id;      // specific ID for the data
    uint16_t   payload_len;  // Length of the following data
    uint8_t    payload[1024];// Max payload buffer
};

struct MotorListPayload {
    uint8_t count;
    uint8_t motor_ids[16]; // Variable length based on count
};

struct CanBusPayload {
    uint8_t interface_id; // 0 = CAN0, 1 = CAN1, etc.
    uint8_t error_code;   // Optional specific CAN error
};

#pragma pack(pop)

class HeartbeatLink {
public:
    // Callback signature: (Data ID, Pointer to Payload, Length)
    using DataCallback = std::function<void(uint16_t, const uint8_t*, uint16_t)>;

    HeartbeatLink(uint16_t local_port, const char* remote_ip, uint16_t remote_port);
    ~HeartbeatLink();

    bool init();
    void close_socket();

    // Set the function to call when DATA packets arrive
    void set_data_callback(DataCallback cb);

    void send_heartbeat();
    void send_data(uint16_t id, const void* payload, uint16_t len);
    
    bool spin_once(); 

    // Returns true if valid packets received within HB_TIMEOUT_MS
    bool is_remote_alive() const;
    
    // Returns one-way latency (ms) based on last packet
    uint64_t get_last_latency_ms() const;

private:
    int sockfd = -1;
    uint16_t local_port;
    struct sockaddr_in remote_addr;
    
    std::atomic<uint64_t> last_rx_time {0};
    std::atomic<uint64_t> last_latency {0};
    std::atomic<uint32_t> tx_seq {0};
    
    DataCallback on_data_received;

    // Helpers
    void process_packet(const uint8_t* buffer, size_t len);
    uint64_t current_time_ms() const;
};