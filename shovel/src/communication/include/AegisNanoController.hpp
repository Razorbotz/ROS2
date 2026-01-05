// AegisController.hpp
#pragma once

#include "AegisBase.hpp"

class AegisNanoController : public AegisBase {
public:
    AegisNanoController(rclcpp::Node::SharedPtr node, 
                    HeartbeatLink& link_ref, 
                    std::mutex& mutex_ref, 
                    RemoteStatus& status_ref,
                    bool& sendRawData_ref,
                    SystemStatus& systemStatus_ref
                    );

    void on_packet_received(uint16_t id, const uint8_t* data, uint16_t len);
};