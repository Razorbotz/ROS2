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

class AegisBase {
protected:
    rclcpp::Node::SharedPtr nodeHandle; 
    HeartbeatLink& hb_link;
    std::mutex& comms_mutex;
    RemoteStatus& remoteStatus;
    bool& sendRawData_ref;
    SystemStatus& systemStatus_ref;

public:
    AegisBase(rclcpp::Node::SharedPtr node, 
              HeartbeatLink& link, 
              std::mutex& mutex, 
              RemoteStatus& r_status,
              bool& raw_data, 
              SystemStatus& sys_status)
        : nodeHandle(node), hb_link(link), comms_mutex(mutex), remoteStatus(r_status),
          sendRawData_ref(raw_data), systemStatus_ref(sys_status) {}

    virtual ~AegisBase() = default;

    void sendJoystickAxis(uint8_t which, uint8_t axis, float value);
    void sendJoystickButton(uint8_t which, uint8_t button, uint8_t state);
    void sendJoystickHat(uint8_t which, uint8_t hat, uint8_t value);
    void sendKeyboardEvent(uint32_t keyval, uint8_t state);
    void sendBinaryMessage(BinaryMessage& binMsg);
    void queryControl();
    void alertNotPrimary();
    void alertPrimary();
    void alertSystemStatusChange();
    void acknowledgeSystemStatusChange(bool error);
    void alertSystemShutdown();
    void alertSystemBoot();
    
    virtual void on_packet_received(uint16_t id, const uint8_t* data, uint16_t len) = 0;

    protected:
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