// g++ -o run_full_integration     test_system_integration.cpp     AegisController.cpp     AegisNanoController.cpp     ../src/BinaryMessage.cpp     Heartbeat.cpp     -lgtest -lgtest_main -lpthread -std=c++17 -I. -I../include -DUNIT_TEST
#include "AegisController.hpp"

AegisController::AegisController(rclcpp::Node::SharedPtr node, 
                                 HeartbeatLink& link_ref,  
                                 CanLink& can_ref,
                                 std::mutex& mutex_ref, 
                                 RemoteStatus& status_ref,
                                 bool& rawData_in,
                                 SystemStatus& sysStatus_in,
                                 ErrorCode& errCode_in
                                 )
    : AegisBase(node, link_ref,  can_ref, mutex_ref, status_ref, rawData_in, sysStatus_in, errCode_in)
{
}

void AegisController::verifyCanStatus(const CanHeartbeatPayload& hb) {
    if (hb.system_status == ERROR) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "ALERT: Peer reported ERROR state via CAN!");
        systemStatus_ref = ERROR;
    }

    if (systemStatus_ref == PRIMARY && hb.system_status == PRIMARY) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Error: Both FCs think they are PRIMARY!");
    }
}

void AegisController::onCanDataReceived(const CanDataPayload& payload) {
    if (hb_link.is_remote_alive()) {
        return;
    }

    switch (payload.message_id) {
        case ID_JAXIS_MSG: {
            JoystickAxis msg;
            if (sizeof(msg) <= sizeof(payload.data)) {
                std::memcpy(&msg, payload.data, sizeof(msg));
                std::cout << "[CAN FAILOVER] JoyAxis: " << (int)msg.axis_id << " val: " << msg.value << std::endl;
            }
            break;
        }
        case ID_JBTN_MSG: {
            JoystickButton msg;
            if (sizeof(msg) <= sizeof(payload.data)) {
                std::memcpy(&msg, payload.data, sizeof(msg));
                std::cout << "[CAN FAILOVER] JoyBtn: " << (int)msg.button_id << " state: " << (int)msg.state << std::endl;
            }
            break;
        }
        default:
            break;
    }
}

bool AegisController::isValidTransition(SystemStatus from, SystemStatus to) {
    switch (from) {
        case PRIMARY:
            return (to == PARTIAL_PRIMARY || // Motor/Node lost
                    to == SINGLE_FC       || // Peer lost (HB/500)
                    to == STANDBY);          // Peer asserted PRIMARY

        case PARTIAL_PRIMARY:
            return (to == PRIMARY ||         // Recovery
                    to == STOP);             // Peer lost while in Partial (Critical)
            
        case SINGLE_FC:
            return (to == PRIMARY ||         // Peer returned
                    to == STOP);             // Motor/Node lost while alone
            
        case STANDBY:
            return (to == SINGLE_FC ||       // Peer died, need to take over
                    to == PRIMARY);         // Normal handover
            
        case STOP:
            return (to == PARTIAL_PRIMARY || // Recovered FC2, still missing motor
                    to == SINGLE_FC ||       // Recovered motor, still missing FC2
                    to == ERROR);            // Gave up
            
        case ERROR:
                return false;
        default:
            return false;
    }
}

void AegisController::onEnterState(SystemStatus state) {
    switch (state) {
        case PRIMARY:{
            if(!motorsAuthorized){
                enableMotorAuthorization();
                motorsAuthorized = true;
            }
            break;
        }
        case STANDBY:

            break;
        case PARTIAL_PRIMARY:{
            if(!motorsAuthorized){
                enableMotorAuthorization();
                motorsAuthorized = true;
            }
            break;
        }

        case PARTIAL_SECONDARY:
            // Auto-trigger the alert logic we discussed
            // alert_pilot("System degraded");
            break;

        case CAN_INOP:
        
            break;

        case ERROR:
            // Immediate safety kill
            // disable_all_motors();
            // trigger_audible_alarm();
            break;

        case SAFETY_DEGRADED:

            break;
            
        case STOP:
            // Ensure timers are cleared
            // takeover_timer.cancel();
            break;
    }
}

void AegisController::onExitState(SystemStatus state) {
    switch (state) {
        case PRIMARY:

            break;
        case STANDBY:

            break;
        case PARTIAL_PRIMARY:
            break;

        case PARTIAL_SECONDARY:
            // Auto-trigger the alert logic we discussed
            // alert_pilot("System degraded");
            break;

        case CAN_INOP:
        
            break;

        case ERROR:
            // Immediate safety kill
            // disable_all_motors();
            // trigger_audible_alarm();
            break;

        case SAFETY_DEGRADED:

            break;
            
        case STOP:
            // Ensure timers are cleared
            // takeover_timer.cancel();
            break;
    }
}

void AegisController::on_packet_received(uint16_t id, const uint8_t* data, uint16_t len) {
    RCLCPP_INFO(nodeHandle->get_logger(), "Orin: Received Message ID: %d", id);
    // Packet containing motor speed values
    switch (id) {
        // --- TELEMETRY --- 000s
        case ID_SPEED_MSG: {
            MotorSpeed msg;
            if (parse_packet(data, len, msg, "MotorSpeed")) {
                std::lock_guard<std::mutex> lock(comms_mutex);
                std::cout << "Motor " << (int)msg.motor_id << " set to " << msg.speed << std::endl;
                // publish_speed(msg);
            }
            break;
        }
        case ID_POS_MSG: {
            MotorPosition msg;
            if (parse_packet(data, len, msg, "MotorPosition")) {
                std::lock_guard<std::mutex> lock(comms_mutex);
                std::cout << "Motor " << (int)msg.motor_id << " pos set to " << msg.position << std::endl;
            }
            break;
        }

        case ID_JAXIS_MSG: {
            // Joystick Axis
            JoystickAxis msg;
            if (parse_packet(data, len, msg, "JoyAxis")) {
                // Handle axis
                std::lock_guard<std::mutex> lock(comms_mutex);
                std::cout << "Joystick ID: " << (int)msg.joystick_id << " Axis: " << (int)msg.axis_id << " value: " << msg.value << std::endl;
            }
            break;
        }

        case ID_JBTN_MSG: {
            // Joystick Button
            JoystickButton msg;
            if (parse_packet(data, len, msg, "JoyBtn")) {
                // Handle button
                std::lock_guard<std::mutex> lock(comms_mutex);
                std::cout << "Joystick ID: " << (int)msg.joystick_id << " Button ID: " << (int)msg.button_id << " state: " << (int)msg.state << std::endl;
            }
            break;
        }

        case ID_JHAT_MSG: {
            // Joystick Hat
            JoystickHat msg;
            if (parse_packet(data, len, msg, "JoyHat")) {
                // Handle hat
                std::lock_guard<std::mutex> lock(comms_mutex);
                std::cout << "Joystick ID: " << (int)msg.joystick_id << " Hat ID: " << (int)msg.hat_id << " value: " << (int)msg.value << std::endl;
            }
            break;
        }

        case ID_KEY_MSG: {
            // Keyboard
            KeyboardEvent msg;
            if (parse_packet(data, len, msg, "Keyboard")) {
                // Handle key
                std::lock_guard<std::mutex> lock(comms_mutex);
                std::cout << "Keycode: " << (int)msg.key_code << " state: " << (int)msg.state << std::endl;
            }
            break;
        }

        case ID_BM_MSG: {
            // Complex BinaryMessage
            // 1. Reconstruct list for BinaryMessage constructor
            std::list<uint8_t> byteList;
            for(int i=0; i<len; i++) byteList.push_back(data[i]);

            // 2. Decode
            BinaryMessage receivedMsg(byteList);
            break;
        }

        // --- Control Configuration --- 100s
        case ID_ASSIGN_AUTH: {
            // Request to control specific motors 
            if (len != sizeof(MotorAuthPayload)) {
                return;
            }

            const MotorAuthPayload* payload = reinterpret_cast<const MotorAuthPayload*>(data);
            processRemoteAuth(payload->motor_states);
            setAuthFromRemote(payload->motor_states);
            for (size_t i = 0; i < MAX_MOTORS; i++) {
                bool is_authorized = auth_table[i];
                if (is_authorized) {
                   std::cout << "Orin: Authorized for Motor ID: " << i << std::endl;
                }
            }
            // Send Confirmation (ID 101)
            sendAuthConfirm();
            break;
        }
        
        case ID_CONFIRM_AUTH:{
            // Response to control request from Nano
            const MotorAuthPayload* payload = reinterpret_cast<const MotorAuthPayload*>(data);
            processRemoteAuth(payload->motor_states);
            for (size_t i = 0; i < MAX_MOTORS; i++) {
                bool is_authorized = remote_auth[i];
                if (is_authorized) {
                   std::cout << "Nano Authorized for Motor ID: " << i << std::endl;
                }
            }
            if(checkAuthErrors()){
                std::cout << "ERROR state, need to resolve." << std::endl;
            }
            if(!checkRemoteAuthStatus() && systemStatus_ref != PRIMARY){
                if(!test)
                    requestControl();
            }
            break;
        }

        // --- State & Handshake --- 200s
        case ID_QUERY_CONTROL:
            // Query about who is in control
            if(systemStatus_ref == PRIMARY || systemStatus_ref == PARTIAL_PRIMARY){
                alertPrimary();
            }
            else if(systemStatus_ref == STANDBY || systemStatus_ref == PARTIAL_SECONDARY){
                alertNotPrimary();
            }
            break;

        case ID_STATE_PRIMARY:
            // Response that the sender is in control
            // Because at most one FC can be in charge, need to transition to standby
            if(systemStatus_ref == PRIMARY){
                requestStateTransition(STANDBY);
            }
            if(systemStatus_ref == PARTIAL_PRIMARY){
                requestStateTransition(PARTIAL_SECONDARY);
            }
            break;
            
        case ID_STATE_STANDBY:{
            // Response that the sender is not in control
            // Secondary is not in control, need to transition to be in charge
            if(systemStatus_ref == STANDBY || systemStatus_ref == SINGLE_FC){
                requestStateTransition(PRIMARY);
            }
            if(systemStatus_ref == PARTIAL_SECONDARY){
                requestStateTransition(PARTIAL_PRIMARY);
            }
            break;
        }
            
        case ID_REQ_RETAKE:{
            // Request from Orin to Nano to retake control
            break;
        }
            
        case ID_GRANT_CONTROL:{
            // Response from Nano to Orin to take control
            if(systemStatus_ref == PARTIAL_PRIMARY){
                requestStateTransition(PRIMARY);
            }
            else{
                std::cout << "Orin was granted control, but is currently in state " << systemStatus_ref << std::endl;
            }
            break;
        }
            
        case ID_DENY_CONTROL:
            // Response from Nano to Orin to not take control
            // This will include a message for how many seconds to delay
            std::cout << "Request to take control was denied" << std::endl;
            break;
            
        case ID_LIVENESS_PING:
            //  Query if the other is alive
            sendPong();
            break;
            
        case ID_LIVENESS_PONG:
            // Response to alive query
            break;
            
        case ID_REQ_RELINQUISH:
            // Request from Nano to Orin to relinquish control
            if(canAcceptControl()){
                sendAcceptControl();
            }
            else{
                sendRejectControl();
            }
            break;
            
        case ID_ACCEPT_CONTROL:
            // Accept control of system
            break;
        
        case ID_REJECT_CONTROL:
            // Reject control of the system
            break;
        
        case ID_SYS_STATUS_CHG: {
            // Change in SystemStatus
            bool error = false;
            uint8_t status = data[0];
            if(systemStatus_ref == PRIMARY || systemStatus_ref == PARTIAL_PRIMARY){
                if(status == PRIMARY || status == PARTIAL_PRIMARY){
                    error = true;
                    std::cout << "Error " << std::endl;
                    alertSystemStatusChange();
                    acknowledgeSystemStatusChange(error);
                    requestStateTransition(STANDBY);
                }
            }
            if(systemStatus_ref == PARTIAL_PRIMARY && status == STANDBY){
                acknowledgeSystemStatusChange(error);
                requestStateTransition(PRIMARY);
            }
            acknowledgeSystemStatusChange(error);
            break;
        }
        
        case ID_ACK_STATUS_CHG: {
            // Acknowledge change in SystemStatus
            if (len < 1) return;
            uint8_t err = data[0];
            if(err == 1){
                requestStateTransition(ERROR);
            }
            else{

            }
            break;
        }

        // --- Parameter Exchange --- 300s
        case ID_PARAM_INIT:
            break;

        case ID_PARAM_DATA:
            break;
        
        case ID_PARAM_ACK:
            break;
            
        case ID_PARAM_REJECT:
            break;

        case ID_SYNC_COMPLETE:
            break;

        case ID_READY_OP:
            break;
        
        // --- Operational Faults & Stops --- 400s
        case ID_ESTOP_HARD:
            // Message to stop immediately
            break;
            
        case ID_ESTOP_SOFT:
            // Message to stop gracefully
            break;
            
        case ID_LOST_MOTORS:{
            // Message from the sender that it lost control of motors
            // Include a list of lost motor IDs
            //processLostMotor();
            break;
        }
        
        case ID_REGAINED_MOTORS:{
            //processRegainedMotor();
            break;
        }
            
        case ID_WIFI_LOST:
            // Lost Wi-Fi connection
            // Nano lost wifi connection, need to send all received data to it
            {
                std::lock_guard<std::mutex> lock(comms_mutex); 
                remoteStatus.WIFI_UP = false;
                sendRawData_ref = true;
            }
            RCLCPP_WARN(nodeHandle->get_logger(), "Nano Wi-Fi is down");
            // Send ACK with ID 406
            acknowledgeWifiChange();
            break;

        case ID_WIFI_REGAINED:
            // Regained Wi-Fi connection
            // Nano regained wifi connection, no need to send all received data to it
            {
                std::lock_guard<std::mutex> lock(comms_mutex); 
                remoteStatus.WIFI_UP = true;
                sendRawData_ref = false;
            }
            RCLCPP_INFO(nodeHandle->get_logger(), "Nano Wi-Fi is up");
            // Send ACK with ID 406
            acknowledgeWifiChange();
            break;

        case ID_WIFI_CONFIRM:
            // Wi-Fi Mode Change Confirm
            break;

        case ID_CAN_DOWN: { 
            // CAN Bus Down
            if (len != 2 * sizeof(uint8_t)) { 
                RCLCPP_ERROR(nodeHandle->get_logger(), "Packet 502 wrong size. Expected 2, got %d", len);
                break;
            }

            struct {
                uint8_t interface_id;
                uint8_t error_code;
            } payload;
            
            std::memcpy(&payload, data, 2);

            {
                std::lock_guard<std::mutex> lock(comms_mutex); 
                
                if (payload.interface_id == 0) remoteStatus.CAN0_UP = false;
                if (payload.interface_id == 1) remoteStatus.CAN1_UP = false;
                
                RCLCPP_WARN(nodeHandle->get_logger(), "CRITICAL: Remote CAN Interface %d DOWN (Error: %d)", payload.interface_id, payload.error_code);
            }
            // Send ACK with ID 409
            break;
        }
            
        case ID_CAN_UP: { 
            // CAN Bus back up
            if (len != 2 * sizeof(uint8_t)) { 
                RCLCPP_ERROR(nodeHandle->get_logger(), "Packet 503 wrong size. Expected 2, got %d", len);
                break;
            }

            struct {
                uint8_t interface_id;
                uint8_t error_code;
            } payload;
            
            std::memcpy(&payload, data, 2);

            {
                std::lock_guard<std::mutex> lock(comms_mutex); 
                
                if (payload.interface_id == 0) remoteStatus.CAN0_UP = true;
                if (payload.interface_id == 1) remoteStatus.CAN1_UP = true;
                
                RCLCPP_INFO(nodeHandle->get_logger(), "CRITICAL: Remote CAN Interface %d UP (Error: %d)", payload.interface_id, payload.error_code);
            }
            // Send ACK with ID 409
            break;
        }
        case ID_CAN_CONFIRM:
            // CAN Bus Mode Change Confirm
            break;

        case ID_SAFE_VIOL_SPD:
            break;
        
        case ID_SAFE_VIOL_POS:
            break;
        
        case ID_SAFE_CONFIRM:
            break;

        case ID_CAN_HB_LOST:
            break;
           
        case ID_CAN_HB_REGAINED:
            break;
            
        case ID_CAN_ACK_CHG:
            break;
            
        case ID_ETH_HB_LOST:
            break;
            
        case ID_ETH_HB_REGAINED:
            break;
            
        case ID_ETH_ACK_HB_CHG:
            break;
            
        case ID_ETH_LOST:
            break; 
        
        case ID_ETH_REGAINED:
            break; 

        case ID_ETH_ACK_CHG:
            break; 

        case ID_MOTORS_INIT:{
            if (len != sizeof(MotorAuthPayload)) {
                return;
            }

            const MotorAuthPayload* payload = reinterpret_cast<const MotorAuthPayload*>(data);
            processRemoteControl(payload->motor_states);
            acknowledgeMotorsDetected();

            if(systemStatus_ref == PRIMARY){
                enableMotorAuthorization();
                sendAuth();
            }
            if(checkControlErrors()){
                requestStateTransition(STOP);
            }
            break;
        }

        case ID_MOTORS_ACK:{
            
            break;
        }
        
        case ID_NODE_LOST: {
            break;
        }

        case ID_NODE_REGAINED: {
            break;
        }

        case ID_NODE_ACK_CHG: {
            break;
        }

        // --- System & Critical Hardware --- 500s
        case ID_SYS_SHUTDOWN:
            // System shutting down
            {
                std::lock_guard<std::mutex> lock(comms_mutex); 
                remoteStatus.UP = false;
            }
            requestStateTransition(SINGLE_FC);
            RCLCPP_WARN(nodeHandle->get_logger(), "Nano shutting down");
            break;
            
        case ID_SYS_BOOT_OK: {
            // System functioning again
            {
                std::lock_guard<std::mutex> lock(comms_mutex); 
                remoteStatus.UP = true;
            }
            if(systemStatus_ref == SINGLE_FC){
                requestStateTransition(PRIMARY);
            }
            queryControl();
            RCLCPP_INFO(nodeHandle->get_logger(), "Nano Booted");
            if(!init_timer_active){
                init_start_time = std::chrono::steady_clock::now();
                init_timer_active = true;
            }
            break;    
        }
    
        default:
            RCLCPP_WARN(nodeHandle->get_logger(), "Received unknown Message ID: %d", id);
            break;
    }
}
