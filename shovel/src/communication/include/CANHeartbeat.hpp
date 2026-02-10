#pragma once
#include <cstdint>
#include <atomic>
#include <cstring>
#include <linux/can.h>
#include "Heartbeat.hpp"

// IDs will be BASE + SenderID. 
constexpr uint32_t CAN_ID_HEARTBEAT_BASE = 0x100;
constexpr uint32_t CAN_ID_DATA_BASE      = 0x200;
constexpr uint64_t CAN_HB_TIMEOUT_MS     = 50;

constexpr uint32_t MFR_CUSTOM       = 15; 
constexpr uint32_t DEV_TYPE_FC      = 10; 

constexpr uint32_t API_CLASS_STATUS = 1;  
constexpr uint32_t API_CLASS_CONTROL= 2;  

constexpr uint32_t API_IDX_HB       = 1;  
constexpr uint32_t API_IDX_DATA     = 2;  

constexpr int SHIFT_DEV_TYPE = 24;
constexpr int SHIFT_MFR      = 16;
constexpr int SHIFT_API_CLS  = 10;
constexpr int SHIFT_API_IDX  = 6;
constexpr int SHIFT_DEV_ID   = 0;

#pragma pack(push, 1)
struct CanHeartbeatPayload {
    uint16_t seq_counter;   
    uint8_t system_status;  
    uint8_t system_flags;   
    uint32_t reserved;      
};

struct CanDataPayload {
    uint16_t message_id;    
    uint8_t data[6];        
};
#pragma pack(pop)

class CanLink {
public:
    CanLink(const char* interface_name = "can0");
    ~CanLink();

    bool init();
    void close_socket();

    void send_heartbeat(uint8_t sender_id, uint8_t sys_status, uint8_t system_flags);
    void send_data(uint8_t sender_id, uint16_t message_id, const void* payload, uint16_t len);

    bool read_raw_frame(struct can_frame& frame);

    bool parse_heartbeat(const struct can_frame& frame, CanHeartbeatPayload& out_hb);
    bool parse_data(const struct can_frame& frame, CanDataPayload& out_data);

    bool read_heartbeat(CanHeartbeatPayload& out_hb);

    bool is_remote_alive() const;
    uint64_t current_time_ms() const;

private:
    int sockfd = -1;
    char ifname[16];
    std::atomic<uint64_t> last_rx_time {0};

    uint32_t build_can_id(uint32_t api_class, uint32_t api_index, uint8_t sender_id) const {
        uint32_t id = 0;
        id |= (DEV_TYPE_FC  & 0x1F) << SHIFT_DEV_TYPE;
        id |= (MFR_CUSTOM   & 0xFF) << SHIFT_MFR;
        id |= (api_class    & 0x3F) << SHIFT_API_CLS;
        id |= (api_index    & 0x0F) << SHIFT_API_IDX;
        id |= (sender_id    & 0x3F) << SHIFT_DEV_ID;
        id |= CAN_EFF_FLAG; 
        return id;
    }
};