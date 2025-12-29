#include "Heartbeat.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <chrono>
#include <fcntl.h>

/**
 * @brief Constructs the Heartbeat Link manager.
 * * Initializes the address structures for the remote peer. Does not open the socket immediately;
 * call init() to begin communication.
 * * @param local_port  The UDP port to bind to on this machine (e.g., 31339).
 * @param remote_ip   The IP address of the target computer (e.g., "10.42.0.2").
 * @param remote_port The UDP port to send packets to on the target (e.g., 31339).
 */
HeartbeatLink::HeartbeatLink(uint16_t local_port, const char* remote_ip, uint16_t remote_port) 
    : local_port(local_port) 
{
    // Initialize Remote Address
    std::memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(remote_port);
    if (inet_pton(AF_INET, remote_ip, &remote_addr.sin_addr) <= 0) {
        std::cerr << "[Heartbeat] Invalid remote IP address: " << remote_ip << std::endl;
    }
}

/**
 * @brief Destructor.
 * * Ensures the socket is properly closed to release OS resources.
 */
HeartbeatLink::~HeartbeatLink() {
    close_socket();
}

/**
 * @brief Closes the active socket descriptor.
 * * Safe to call multiple times; checks if sockfd >= 0 before closing.
 */
void HeartbeatLink::close_socket() {
    if (sockfd >= 0) {
        close(sockfd);
        sockfd = -1;
    }
}

/**
 * @brief Initializes the UDP socket for non-blocking communication.
 * * Performs the following steps:
 * 1. Creates a socket file descriptor (AF_INET, SOCK_DGRAM).
 * 2. Sets the O_NONBLOCK flag so recv() calls do not hang the main loop.
 * 3. Binds the socket to the specified local_port.
 * * @return true if socket creation and binding were successful.
 * @return false if the socket failed to open or bind (check errno/perror output).
 */
bool HeartbeatLink::init() {
    if (sockfd >= 0) close_socket();

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("[Heartbeat] Socket creation failed");
        return false;
    }

    int flags = fcntl(sockfd, F_GETFL, 0);
    if (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("[Heartbeat] Failed to set non-blocking");
        return false;
    }

    // Bind to Local Port
    struct sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(local_port);

    if (bind(sockfd, (const struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        perror("[Heartbeat] Bind failed");
        return false;
    }

    return true;
}

/**
 * @brief Registers a callback function for incoming Data packets.
 * * When spin_once() receives a packet with type MSG_TYPE_DATA (0x02), 
 * this function will be called with the ID and payload.
 * * @param cb A std::function or lambda matching the signature:
 * void(uint16_t id, const uint8_t* payload, uint16_t len)
 */
void HeartbeatLink::set_data_callback(DataCallback cb) {
    on_data_received = cb;
}

/**
 * @brief Internal helper to get monotonic system time.
 * * @return Milliseconds since system boot (std::chrono::steady_clock).
 */
uint64_t HeartbeatLink::current_time_ms() const {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

/**
 * @brief Sends a Heartbeat (Keep-Alive) packet.
 * * Constructs a NanoHeader with type MSG_TYPE_HEARTBEAT (0x01) and the
 * current system timestamp. This should be called at 100Hz (every 10ms).
 * * Thread Safety: Safe to call from a dedicated thread if tx_seq is atomic.
 */
void HeartbeatLink::send_heartbeat() {
    if (sockfd < 0) return;

    NanoHeader hb{};
    hb.magic = HB_MAGIC;
    hb.type = MSG_TYPE_HEARTBEAT;
    hb.version = HB_VER;
    hb.seq = ++tx_seq;
    hb.t_ms = current_time_ms();
    
    sendto(sockfd, &hb, sizeof(hb), 0, (struct sockaddr*)&remote_addr, sizeof(remote_addr));
}

/**
 * @brief Sends a generic Data packet (e.g., Motor Speed, CAN Error).
 * * Wraps the payload in a NanoDataPacket structure, calculates the total size,
 * and transmits it to the remote peer.
 * * @param id      The specific Data ID (e.g., 001 for Speed, 502 for CAN Down).
 * @param payload Pointer to the raw data structure to send.
 * @param len     Size of the payload in bytes (Must be <= 1024).
 */
void HeartbeatLink::send_data(uint16_t id, const void* payload, uint16_t len) {
    if (sockfd < 0) return;
    if (len > 1024) {
        std::cerr << "[Heartbeat] Payload too large (" << len << "), dropping." << std::endl;
        return;
    }

    NanoDataPacket packet{};
    
    // Fill Header
    packet.header.magic = HB_MAGIC;
    packet.header.type = MSG_TYPE_DATA;
    packet.header.version = HB_VER;
    packet.header.seq = ++tx_seq;
    packet.header.t_ms = current_time_ms();

    // Fill Data
    packet.data_id = id;
    packet.payload_len = len;
    std::memcpy(packet.payload, payload, len);

    // Calc total size: Header + ID/Len fields (4 bytes) + Actual Payload
    size_t total_size = sizeof(NanoHeader) + sizeof(uint16_t)*2 + len;

    sendto(sockfd, &packet, total_size, 0, (struct sockaddr*)&remote_addr, sizeof(remote_addr));
}


/**
 * @brief Checks for incoming packets and processes them.
 * * This function performs a non-blocking recvfrom() call. It should be called
 * as frequently as possible (e.g., in a "while" loop) to drain the OS buffer.
 * * @return true if a valid packet was received and processed.
 * @return false if the buffer was empty (EAGAIN/EWOULDBLOCK).
 */
bool HeartbeatLink::spin_once() {
    if (sockfd < 0) return false;

    uint8_t buffer[1500];
    struct sockaddr_in sender_addr;
    socklen_t addr_len = sizeof(sender_addr);

    ssize_t len = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&sender_addr, &addr_len);
    
    if (len > 0) {
        process_packet(buffer, static_cast<size_t>(len));
        return true;
    }
    return false;
}

/**
 * @brief Internal helper to parse and validate raw bytes.
 * * 1. Checks packet length vs NanoHeader size.
 * 2. Verifies Magic Number (0xBEEF) and Protocol Version.
 * 3. Updates last_rx_time for liveness tracking.
 * 4. Dispatches Data packets to the registered callback.
 * * @param buffer Pointer to the received data.
 * @param len    Length of the received data.
 */
void HeartbeatLink::process_packet(const uint8_t* buffer, size_t len) {
    // 1. Basic Size Check
    if (len < sizeof(NanoHeader)) return;
    
    const NanoHeader* hdr = reinterpret_cast<const NanoHeader*>(buffer);

    // 2. Magic/Version Check
    if (hdr->magic != HB_MAGIC || hdr->version != HB_VER) return;

    // 3. Update Liveness
    uint64_t now = current_time_ms();
    last_rx_time.store(now);
    
    // Calculate Latency (Clock sync warning: assumes loosely synced clocks or purely relative jitter)
    if (now >= hdr->t_ms) {
        last_latency.store(now - hdr->t_ms);
    }

    // 4. Handle Data
    if (hdr->type == MSG_TYPE_DATA && on_data_received) {
        // Validation: Header + Data Fields
        if (len < (sizeof(NanoHeader) + sizeof(uint16_t)*2)) return;

        const NanoDataPacket* dataParams = reinterpret_cast<const NanoDataPacket*>(buffer);
        
        // Safety: Ensure we don't read past the buffer
        size_t expected_size = sizeof(NanoHeader) + sizeof(uint16_t)*2 + dataParams->payload_len;
        if (len < expected_size) {
            std::cerr << "[Heartbeat] Truncated data packet received." << std::endl;
            return;
        }

        // Fire Callback
        on_data_received(dataParams->data_id, dataParams->payload, dataParams->payload_len);
    }
}

/**
 * @brief Checks if the remote peer is currently considered "Alive".
 * * Based on the time elapsed since the last valid packet was received.
 * * @return true if (CurrentTime - LastRxTime) <= HB_TIMEOUT_MS (50ms).
 * @return false if the timeout has been exceeded or no packets ever received.
 */
bool HeartbeatLink::is_remote_alive() const {
    uint64_t last = last_rx_time.load();
    if (last == 0) return false; // Never heard from them

    uint64_t diff = current_time_ms() - last;
    return diff <= HB_TIMEOUT_MS;
}

/**
 * @brief Returns the one-way latency of the link.
 * * Calculated as (LocalRecvTime - RemoteSendTime) from the last packet.
 * Note: Requires loosely synchronized clocks or assumes relative jitter 
 * is the only metric of interest.
 * * @return Latency in milliseconds.
 */
uint64_t HeartbeatLink::get_last_latency_ms() const {
    return last_latency.load();
}