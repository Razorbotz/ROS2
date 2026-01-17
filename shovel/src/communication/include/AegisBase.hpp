#pragma once

#ifdef UNIT_TEST
    #include "MockDeps.hpp"
#else
    #include <rclcpp/rclcpp.hpp>
#endif
#include "BinaryMessage.hpp"
#include "Heartbeat.hpp" 
#include "CANHeartbeat.hpp"
#include "RobotState.hpp"
#include <mutex>
#include <cstring>
#include <atomic>


// Currently there are 8 motors. They range from 10-17
constexpr size_t MAX_MOTOR_ID = 8;

class AegisBase {
protected:
    rclcpp::Node::SharedPtr nodeHandle; 
    HeartbeatLink& hb_link;
    CanLink& can_link;
    std::mutex& comms_mutex;
    RemoteStatus& remoteStatus;
    bool& sendRawData_ref;
    SystemStatus& systemStatus_ref;
    std::atomic<uint64_t> last_can_rx_time {0};
    
    // This is used to track whether the controller has authorization
    // to control the motor
    std::array<bool, MAX_MOTOR_ID> auth_table;
    // Used to track whether the motors are found on CAN0
    std::array<bool, MAX_MOTOR_ID> can0_table;
    // Used to track whether the motors are found on CAN1
    std::array<bool, MAX_MOTOR_ID> can1_table;
    // Used to track which interface the motor is currently attached to
    std::array<bool, MAX_MOTOR_ID> can_table;
    // Used to track whether the remote controller can control the motor
    std::array<bool, MAX_MOTOR_ID> remote_auth;
    // Used to track whether the remote controller can control the motor
    std::array<bool, MAX_MOTOR_ID> remote_cont;
    bool alertedRemoteMotors = false;
    bool motorsAuthorized = false;

public:
    AegisBase(rclcpp::Node::SharedPtr node, 
              HeartbeatLink& link, 
              CanLink& c_link,
              std::mutex& mutex, 
              RemoteStatus& r_status,
              bool& raw_data, 
              SystemStatus& sys_status)
        : nodeHandle(node), hb_link(link), can_link(c_link), comms_mutex(mutex), remoteStatus(r_status),
          sendRawData_ref(raw_data), systemStatus_ref(sys_status) {
            auth_table.fill(false);
            can0_table.fill(false);
            can1_table.fill(false);
            can_table.fill(false);
            remote_auth.fill(false);
            remote_cont.fill(false);
          }

    virtual ~AegisBase() = default;

    // 000s
    void sendJoystickAxis(uint8_t which, uint8_t axis, float value);
    void sendJoystickButton(uint8_t which, uint8_t button, uint8_t state);
    void sendJoystickHat(uint8_t which, uint8_t hat, uint8_t value);
    void sendKeyboardEvent(uint32_t keyval, uint8_t state);
    void sendBinaryMessage(BinaryMessage& binMsg);

    // 100s
    void sendAuth();
    void sendAuthConfirm();

    // 200s
    void queryControl();
    void alertPrimary();
    void alertNotPrimary();
    void requestControl();
    void grantControl();
    void denyControl();
    void sendPing();
    void sendPong();
    void alertSystemStatusChange();
    void acknowledgeSystemStatusChange(bool error);

    // 300s

    // 400s
    void alertLostMotor(uint8_t motor_id);
    void alertRegainedMotor(uint8_t motor_id);
    void alertWifiLost();
    void alertWifiRegained();
    void acknowledgeWifiChange();
    void alertMotorsDetected();
    void acknowledgeMotorsDetected();

    // 500s
    void alertSystemShutdown();
    void alertSystemBoot();

    // Helper functions
    bool isMotorAuthorized(uint8_t motor_id) const;
    void updateMotorAuthorization(uint8_t motor_id, bool authorized);
    bool isMotorDetectedCAN0(uint8_t motor_id);
    void updateMotorCAN0State(uint8_t motor_id, bool up);
    bool isMotorDetectedCAN1(uint8_t motor_id);
    void updateMotorCAN1State(uint8_t motor_id, bool up);

    /**
     * This function sets the authorization of the PRIMARY controller
     * based on whether the motors can be detected on either CAN
     * interface.
     */
    void enableMotorAuthorization();
    /**
     * This function takes the authorization array from the remote controller
     * and sets the remote_auth values based on the received values. 
     */
    bool processRemoteAuth(const uint8_t motor_states[MAX_MOTORS]);
    /**
     * This function is used to set the authorization of the controller based
     * on the other. This will be called by the secondary flight controller to 
     * ensure that only one controller is responsible for commanding motors at any
     * time. 
     */
    void setAuthFromRemote(const uint8_t motor_states[MAX_MOTORS]);
    /**
     * This function is used to ensure that both motor controllers aren't attempting
     * to have authorization for a motor. If both 
     */
    bool checkAuth();
    /**
     * This function is used to check whether the controller is authorized
     * to control any motors. 
     * 
     * @return Boolean value of whether any motors are authorized to be controlled 
     */
    bool checkAuthStatus();
    /**
     * This function is used to check whether the remote controller is authorized
     * to control any motors. 
     * 
     * @return Boolean value of whether any motors are authorized to be controlled by
     * the remote controller
     */
    bool checkRemoteAuthStatus();
    void processRemoteControl(const uint8_t motor_states[MAX_MOTORS]);
    void processLostMotor(const uint8_t motor_states[MAX_MOTORS]);

    bool checkAllMotorsInit();

    void checkMotorInitTimer();

    std::chrono::steady_clock::time_point init_start_time;
    bool init_timer_active = false;

    virtual void on_packet_received(uint16_t id, const uint8_t* data, uint16_t len) = 0;
    
    virtual void onCanHeartbeatReceived(const CanHeartbeatPayload& hb) {
        last_can_rx_time.store(can_link.current_time_ms());
        verifyCanStatus(hb);
    }

    virtual void onCanDataReceived(const CanDataPayload& data) = 0;

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

    virtual void verifyCanStatus(const CanHeartbeatPayload& hb) {}
};