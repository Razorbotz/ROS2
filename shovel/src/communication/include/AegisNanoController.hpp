#pragma once

#include "AegisBase.hpp"

class AegisNanoController : public AegisBase {
public:
    AegisNanoController(rclcpp::Node::SharedPtr node, 
                    HeartbeatLink& link_ref,  
                    CanLink& can_ref,
                    std::mutex& mutex_ref, 
                    RemoteStatus& status_ref,
                    bool& sendRawData_ref,
                    SystemStatus& systemStatus_ref,
                    ErrorCode& errorCode_ref
                    );

    void on_packet_received(uint16_t id, const uint8_t* data, uint16_t len);
    void checkTimers() override;
    void checkTakeoverTimer(); 
    void checkAuthorityTimer();
    void verifyCanStatus(const CanHeartbeatPayload& hb) override;
    void onCanDataReceived(const CanDataPayload& payload) override;
    void onEnterState(SystemStatus state) override;
    void onExitState(SystemStatus state) override;

    private:
        std::chrono::steady_clock::time_point takeover_start_time;
        bool takeover_timer_active = false;
        std::chrono::steady_clock::time_point relinquish_start_time;
        bool relinquish_timer_active = false;
};