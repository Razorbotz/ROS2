#include "CANHeartbeat.hpp"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <chrono>
#include <cstring>

CanLink::CanLink(const char* interface_name) {
    std::strncpy(ifname, interface_name, sizeof(ifname));
}

CanLink::~CanLink() { close_socket(); }

void CanLink::close_socket() {
    if (sockfd >= 0) {
        close(sockfd);
        sockfd = -1;
    }
}

bool CanLink::init() {
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0) {
        perror("SocketCAN: Error opening socket");
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) {
        perror("SocketCAN: Error finding interface index");
        close_socket();
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("SocketCAN: Error binding socket");
        close_socket();
        return false;
    }

    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    return true; 
}

void CanLink::send_heartbeat(uint8_t sender_id, uint8_t sys_status, uint8_t system_flags) {
    if (sockfd < 0) return;

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    frame.can_id = build_can_id(API_CLASS_STATUS, API_IDX_HB, sender_id);
    frame.can_dlc = sizeof(CanHeartbeatPayload);

    CanHeartbeatPayload payload;
    static uint16_t seq = 0;
    payload.seq_counter = seq++;
    payload.system_status = sys_status;
    payload.system_flags = system_flags;
    payload.reserved = 0;

    std::memcpy(frame.data, &payload, sizeof(payload));
    write(sockfd, &frame, sizeof(frame));
}

void CanLink::send_data(uint8_t sender_id, uint16_t message_id, const void* payload, uint16_t len){
    if (sockfd < 0) return;
    if (len > 6) return;

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    frame.can_id = build_can_id(API_CLASS_CONTROL, API_IDX_DATA, sender_id);
    frame.can_dlc = sizeof(CanDataPayload);

    CanDataPayload dataMsg;
    dataMsg.message_id = message_id;
    std::memset(dataMsg.data, 0, 6);
    if (payload && len > 0) std::memcpy(dataMsg.data, payload, len);

    std::memcpy(frame.data, &dataMsg, sizeof(dataMsg));
    write(sockfd, &frame, sizeof(frame));
}

bool CanLink::read_raw_frame(struct can_frame& frame) {
    if (sockfd < 0) return false;
    
    int nbytes = read(sockfd, &frame, sizeof(struct can_frame));
    if (nbytes < 0) return false;
    if (nbytes < (int)sizeof(struct can_frame)) return false; 
    return true;
}

bool CanLink::parse_heartbeat(const struct can_frame& frame, CanHeartbeatPayload& out_hb) {
    if (!(frame.can_id & CAN_EFF_FLAG)) return false;

    uint32_t id = frame.can_id & CAN_EFF_MASK;
    uint32_t api_class = (id >> SHIFT_API_CLS) & 0x3F;
    uint32_t api_idx   = (id >> SHIFT_API_IDX) & 0x0F;
    uint32_t mfr       = (id >> SHIFT_MFR)     & 0xFF;

    if (mfr != MFR_CUSTOM) return false;
    if (api_class != API_CLASS_STATUS) return false;
    if (api_idx != API_IDX_HB) return false;
    
    if (frame.can_dlc != sizeof(CanHeartbeatPayload)) return false;
    std::memcpy(&out_hb, frame.data, sizeof(CanHeartbeatPayload));
    
    last_rx_time.store(current_time_ms());
    return true; 
}

bool CanLink::parse_data(const struct can_frame& frame, CanDataPayload& out_data) {
    if (!(frame.can_id & CAN_EFF_FLAG)) return false;

    uint32_t id = frame.can_id & CAN_EFF_MASK;
    uint32_t api_class = (id >> SHIFT_API_CLS) & 0x3F;
    uint32_t api_idx   = (id >> SHIFT_API_IDX) & 0x0F;
    uint32_t mfr       = (id >> SHIFT_MFR)     & 0xFF;

    if (mfr != MFR_CUSTOM) return false;
    if (api_class != API_CLASS_CONTROL) return false;
    if (api_idx != API_IDX_DATA) return false;

    if (frame.can_dlc != sizeof(CanDataPayload)) return false;
    std::memcpy(&out_data, frame.data, sizeof(CanDataPayload));
    return true;
}

bool CanLink::read_heartbeat(CanHeartbeatPayload& out_hb) {
    struct can_frame frame;
    if (read_raw_frame(frame)) {
        return parse_heartbeat(frame, out_hb);
    }
    return false;
}

uint64_t CanLink::current_time_ms() const {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

bool CanLink::is_remote_alive() const {
    uint64_t last = last_rx_time.load();
    if (last == 0) return false; 
    uint64_t diff = current_time_ms() - last;
    return diff <= CAN_HB_TIMEOUT_MS;
}