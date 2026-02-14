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
                    HandshakeStatus& handshakeStatus_ref,
                    ErrorCode& errorCode_ref,
                    std::function<void(bool)> callback = nullptr,
                    std::function<void(uint8_t motor_index, bool authorized)> update_motor_auth = nullptr
                    );

    std::array<bool, MAX_MOTOR_ID> deferred_release;
    bool has_deferred_release = false;

    void checkDeferredRelease();
    int handshake_step = 0;
    SimpleTimer retry_timer; 
    void advanceHandshake();
    void initiateHandshakeState(uint16_t id);
    void processHandshakePacket(uint16_t id, const uint8_t* data);
    void handleSystemStatusOverride(const uint8_t* data);
    void handleParamExchange(uint16_t id);
    void handleMotorAuth(uint16_t id, const uint8_t* data);
    void on_packet_received(uint16_t id, const uint8_t* data, uint16_t len);
    
    /**
     * @brief Returns true if the Nano is in a state where it should
     * process motor authorization changes from message 100 (ASSIGN_AUTH).
     * Guards against auth changes while in STOP or other non-operational states.
     */
    bool canProcessAuthAssignment() const;
    
    void checkTimers();
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