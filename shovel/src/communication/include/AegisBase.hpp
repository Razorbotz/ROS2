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
#include "SimpleTimer.hpp"


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
    HandshakeStatus& handshakeStatus_ref;
    ErrorCode& errorCode_ref;
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
    // Used to track whether the motor nodes are functional
    std::array<bool, MAX_MOTOR_ID + 6> node_table;
    // Used to track whether the remote controller can control the motor
    std::array<bool, MAX_MOTOR_ID> remote_auth;
    // Used to track whether the remote controller can control the motor
    std::array<bool, MAX_MOTOR_ID> remote_cont;
    bool alertedRemoteMotors = false;
    bool motorsAuthorized = false;
    SimpleTimer boot_timer; 
    bool boot_checks_passed = false;
    std::atomic<bool> remote_shutdown_latched{false};
    std::array<float, MAX_MOTOR_ID> motor_speeds;

    SimpleTimer motor10NodeTimer;
    bool motor10NodeActive = false;
    SimpleTimer motor11NodeTimer;
    bool motor11NodeActive = false;
    SimpleTimer motor12NodeTimer;
    bool motor12NodeActive = false;
    SimpleTimer motor13NodeTimer;
    bool motor13NodeActive = false;
    SimpleTimer motor14NodeTimer;
    bool motor14NodeActive = false;
    SimpleTimer motor15NodeTimer;
    bool motor15NodeActive = false;
    SimpleTimer motor16NodeTimer;
    bool motor16NodeActive = false;
    SimpleTimer motor17NodeTimer;
    bool motor17NodeActive = false;
    SimpleTimer logicNodeTimer;
    bool logicNodeActive = false;
    SimpleTimer autonomyNodeTimer;
    bool autonomyNodeActive = false;
    SimpleTimer excavationNodeTimer;
    bool excavationNodeActive = false;
    SimpleTimer statusMonitorNodeTimer;
    bool statusMonitorNodeActive = false;
    SimpleTimer videoStreamNodeTimer;
    bool videoStreamNodeActive = false;
    SimpleTimer zedTrackingNodeTimer;
    bool zedTrackingNodeActive = false;

public:
    AegisBase(rclcpp::Node::SharedPtr node, 
              HeartbeatLink& link, 
              CanLink& c_link,
              std::mutex& mutex, 
              RemoteStatus& r_status,
              bool& raw_data, 
              SystemStatus& sys_status,
              HandshakeStatus& hand_status,
              ErrorCode& error_code)
        : nodeHandle(node), hb_link(link), can_link(c_link), comms_mutex(mutex), remoteStatus(r_status),
          sendRawData_ref(raw_data), systemStatus_ref(sys_status), handshakeStatus_ref(hand_status), errorCode_ref(error_code) {
            auth_table.fill(false);
            can0_table.fill(false);
            can1_table.fill(false);
            can_table.fill(false);
            node_table.fill(false);
            remote_auth.fill(false);
            remote_cont.fill(false);
            remoteStatus.STATUS = BOOT;
            motor_speeds.fill(0.0);
          }

    virtual ~AegisBase() = default;

    void initAegis();
    void checkBootTimer();
    void checkNodeTimers();

    void requestStateTransition(SystemStatus new_state);

    bool isValidTransition(SystemStatus from, SystemStatus to);
    virtual void onEnterState(SystemStatus state) = 0;
    virtual void onExitState(SystemStatus state) = 0;
    std::string stateToString(SystemStatus state);

    void receivedMotor10();
    void receivedMotor11();
    void receivedMotor12();
    void receivedMotor13();
    void receivedMotor14();
    void receivedMotor15();
    void receivedMotor16();
    void receivedMotor17();
    void receivedLogic();
    void receivedAutonomy();
    void receivedExcavation();
    void receivedStatusMonitor();
    void receivedVideoStream();
    void receivedZedTracking();

    // 000s
    void sendSpeedMessage();
    void sendPositionMessage();
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
    void sendRelinquishRequest();
    void sendAcceptControl();
    void sendRejectControl();
    void alertSystemStatusChange(bool verbose = true);
    void acknowledgeSystemStatusChange(bool error);

    // 300s
    void sendParamInit();
    void sendParamData();
    void sendParamAck();
    void sendParamReject();
    void sendSyncComplete();
    void sendReadyOp();

    // 400s
    void sendHardEStop();
    void sendSoftEStop();
    void alertLostMotor(uint8_t motor_id);
    void alertRegainedMotor(uint8_t motor_id);
    void alertWifiLost();
    void alertWifiRegained();
    void acknowledgeWifiChange();
    void alertMotorsDetected();
    void acknowledgeMotorsDetected();
    void alertLostNode(uint8_t node_lost);
    void alertRegainedNode(uint8_t node_regained);
    void acknowledgeNodeChange();

    // 500s
    void alertSystemShutdown();
    void alertSystemBoot();
    void alertSystemBootAck();

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
    bool checkAuthErrors();
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
    void processRegainedMotor(const uint8_t motor_states[MAX_MOTORS]);
    void processLostNode(uint8_t node);
    void processRegainedNode(uint8_t node);

    bool checkAllMotorsInit();

    bool canAcceptControl();
    bool canGiveControl();

    bool checkControlErrors();
    void checkMotorControlStatus();
    void applyRemoteAlivePolicy();
    bool checkRemoteAlive();
    bool isHandshakeMsg(uint16_t id);

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