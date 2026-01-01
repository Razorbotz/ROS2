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
 * 0xx: Telemetry & Setpoints
 * 001: Motor speed values
 *    - This will have motor ID and speed value (float 32)
 * 002: Motor position value
 *     - This will have motor ID and position (int32) 
 * 
 * 1xx: Resource Ownership & Delegation
 * Commands to change the configuration of the robot.
 * NOTE: This DOES NOT change the control status, only individual motor status
 * 100: Message from the Orin to the Nano to control motors
 *     - This will include a message with the motors to control
 *     - This will be a list of motor IDs to control
 * 101: Confirmation of motor control from Nano to Orin
 * 
 * 2xx: State & Handshake
 * Negotiating who is in charge.
 * 200: Query about who is in control
 * 201: Response that the sender is in control
 * 202: Response that the sender is not in control
 * 203: Request from Orin to Nano to retake control
 * 204: Response from Nano to Orin to take control
 * 205: Response from Nano to Orin to not take control
 *     - This will include a message for how many seconds to delay
 * 206: Query if the other is alive
 * 207: Response to alive query
 * 208: Request from Nano to Orin to relinquish control
 * 209: Accept control of system

 * 4xx: Operational Faults & Stops
 * Interface issues
 * 400: Message to force stop immediately
 * 401: Message to stop gracefully
 * 402: Message from the sender that it lost control of motors
 *     - Include a list of lost motor IDs
 * 403: Message from sender that it has regained control
 *     - Include list of regained motor IDs
 * 404: Lost Wi-Fi connection
 * 405: Regained Wi-Fi connection
 * 406: Wi-Fi Mode Change Confirm
 * 407: CAN Bus is down
 * 408: CAN Bus is up
 * 409: CAN Bus Mode Change Confirm

 * 5xx: System & Critical Hardware
 * System level errors and shutdown commands
 * 500: System shutting down
 * 501: System functioning again
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

struct MotorSpeed {
    uint8_t motor_id;
    float speed;
};

struct MotorPosition {
    uint8_t motor_id;
    int32_t position;
};

struct CanBusPayload {
    uint8_t interface_id; // 0 = CAN0, 1 = CAN1, etc.
    uint8_t error_code;   // Optional specific CAN error
};

struct RemoteStatus {
    bool UP;
    bool WIFI_UP;
    bool CAN0_UP;
    bool CAN1_UP;
};

struct JoystickAxis {
    uint8_t joystick_id;
    uint8_t axis_id;
    float   value;
};

struct JoystickButton {
    uint8_t joystick_id;
    uint8_t button_id;
    uint8_t state; // 0=Release, 1=Press
};

struct JoystickHat {
    uint8_t joystick_id;
    uint8_t hat_id;
    uint8_t value; // Hat direction code
};

struct KeyboardEvent {
    uint32_t key_code;
    uint8_t  state; // 0=Release, 1=Press
};

enum SystemStatus {
    PRIMARY, // Should control all motors and send data to client
    STANDBY, // Should act as safety monitor and backup
    SINGLE_FC, // Only acting FC, should be more careful
    PARTIAL_PRIMARY, // Should send data to client, controls part of motors
    PARTIAL_SECONDARY, // Controls part of motors, only send those motors to client,
    CAN_INOP, // 
    ERROR, // 
    SAFETY_DEGRADED,
    STOP // STOP
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