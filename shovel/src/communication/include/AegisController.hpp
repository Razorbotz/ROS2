#pragma once
#include "AegisBase.hpp"

class AegisController : public AegisBase {
public:
    AegisController(rclcpp::Node::SharedPtr node, 
                    HeartbeatLink& link_ref, 
                    CanLink& can_ref,
                    std::mutex& mutex_ref, 
                    RemoteStatus& status_ref,
                    bool& sendRawData_ref,
                    SystemStatus& systemStatus_ref,
                    HandshakeStatus& handshakeStatus_ref,
                    ErrorCode& errorCode_ref,
                    std::function<void(bool)> callback = nullptr
                    );
    
    SimpleTimer auth_request_timer;
    int auth_request_attempts = 0;
    static constexpr int AUTH_REQUEST_BASE_MS = 200;
    static constexpr int AUTH_REQUEST_MAX_BACKOFF_MS = 5000;
    static constexpr int AUTH_REQUEST_BACKOFF_THRESHOLD = 3;
    bool auth_request_pending = false;

    void checkAuthRequestTimer();
    int getAuthRequestDelay() const;
    int handshake_step = 0;
    SimpleTimer retry_timer; 
    bool hasControl = false;
    bool canTakeControl();
    void advanceHandshake();
    void processHandshakePacket(uint16_t id, const uint8_t* data);
    void handleControlStep(uint16_t id, const uint8_t* data, bool& step_complete);
    void handleParamStep(uint16_t id, bool& step_complete);
    void handleMotorStep(uint16_t id, bool& step_complete);
    void checkTimers();
    void on_packet_received(uint16_t id, const uint8_t* data, uint16_t len);
    void verifyCanStatus(const CanHeartbeatPayload& hb) override;
    void onCanDataReceived(const CanDataPayload& payload) override;
    void onEnterState(SystemStatus state) override;
    void onExitState(SystemStatus state) override;
    bool test = false;
};