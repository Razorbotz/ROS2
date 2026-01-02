// AegisController.hpp
#pragma once

#ifdef UNIT_TEST
    #include "MockDeps.hpp"
#else
    #include <rclcpp/rclcpp.hpp>
#endif
#include "BinaryMessage.hpp"
#include "Heartbeat.hpp" 
#include "RobotState.hpp"
#include <mutex>
#include <cstring>

class AegisNanoController {
public:
    AegisNanoController(rclcpp::Node::SharedPtr node, 
                    HeartbeatLink& link_ref, 
                    std::mutex& mutex_ref, 
                    RemoteStatus& status_ref,
                    bool& sendRawData_ref,
                    SystemStatus& systemStatus_ref
                    );

    void sendJoystickAxis(uint8_t which, uint8_t axis, float value) {
        if (!hb_link.is_remote_alive()) return;
        JoystickAxis msg {which, axis, value};
        hb_link.send_data(010, &msg, sizeof(msg));
    }

    void sendJoystickButton(uint8_t which, uint8_t button, uint8_t state) {
        if (!hb_link.is_remote_alive()) return;
        JoystickButton msg {which, button, state};
        hb_link.send_data(011, &msg, sizeof(msg));
    }

    void sendJoystickHat(uint8_t which, uint8_t hat, uint8_t value) {
        if (!hb_link.is_remote_alive()) return;
        JoystickHat msg {which, hat, value};
        hb_link.send_data(012, &msg, sizeof(msg));
    }

    void sendKeyboardEvent(uint32_t keyval, uint8_t state) {
        if (!hb_link.is_remote_alive()) return;
        KeyboardEvent msg {keyval, state};
        hb_link.send_data(013, &msg, sizeof(msg));
    }

    void sendBinaryMessage(BinaryMessage& binMsg) {
        if (!hb_link.is_remote_alive()) return;
        auto bytesList = binMsg.getBytes();
        std::vector<uint8_t> buffer(bytesList->begin(), bytesList->end());
        hb_link.send_data(020, buffer.data(), buffer.size());
    }

    void on_packet_received(uint16_t id, const uint8_t* data, uint16_t len);

private:
    rclcpp::Node::SharedPtr nodeHandle; 
    HeartbeatLink& hb_link;
    std::mutex& comms_mutex;
    RemoteStatus& nanoStatus;
    bool& sendRawData_ref;
    SystemStatus& systemStatus_ref;

    template <typename T>
    static bool parse_packet(const uint8_t* data, uint16_t len, T& out_struct, const char* name) {
        if (len != sizeof(T)) {
            std::cerr << "Error: Malformed " << name << " packet. Expected " 
                      << sizeof(T) << " bytes, got " << len << std::endl;
            return false;
        }
        std::memcpy(&out_struct, data, sizeof(T));
        return true;
    }
};