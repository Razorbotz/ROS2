#include "AegisBase.hpp"

// AegisBase.cpp

void AegisBase::initAegis(){
    systemStatus_ref = BOOT;    
    std::cout << "[System] Booting... Waiting 1s for peripherals." << std::endl;
    boot_timer.start(1000); 
}

void AegisBase::checkBootTimer() {
    if (systemStatus_ref == BOOT) {
        if (boot_timer.isExpired()) {
            alertSystemBoot();
            requestStateTransition(STANDBY);
        }
    }
}


void AegisBase::requestStateTransition(SystemStatus new_state) {
    if (systemStatus_ref == new_state) {
        return;
    }

    if(new_state == SINGLE_FC){
        if(systemStatus_ref == PARTIAL_PRIMARY){
            requestStateTransition(STOP);
            return;
        }
    }

    if (!isValidTransition(systemStatus_ref, new_state)) {
        std::cerr << "[FSM] ILLEGAL TRANSITION ATTEMPT: " 
                  << stateToString(systemStatus_ref) << " -> " 
                  << stateToString(new_state) << std::endl;
        return;
    }

    std::cout << "[FSM] Transition: " << stateToString(systemStatus_ref) 
              << " -> " << stateToString(new_state) << std::endl;

    onExitState(systemStatus_ref);
    SystemStatus prev = systemStatus_ref;
    systemStatus_ref = new_state;
    onEnterState(new_state);
    if(new_state != SINGLE_FC && prev != SINGLE_FC){
        if(!(new_state == STANDBY && prev == BOOT)){
            alertSystemStatusChange();
        }
    }
}

bool AegisBase::isValidTransition(SystemStatus from, SystemStatus to) {
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
                    to == PRIMARY   ||       // Normal handover
                    to == PARTIAL_SECONDARY ||
                    to == STOP);
            
        case STOP:
            return (to == PARTIAL_PRIMARY || // Recovered FC2, still missing motor
                    to == SINGLE_FC ||       // Recovered motor, still missing FC2
                    to == ERROR ||           // Gave up
                    to == STANDBY);          // Entered standby  
            
        case PARTIAL_SECONDARY:
            return (to == STANDBY ||
                    to == SINGLE_FC ||
                    to == STOP);
            
        case ERROR:
                return false;

        case BOOT:
                return (to == STANDBY);
        default:
            return false;
    }
}

std::string AegisBase::stateToString(SystemStatus state) {
    switch (state) {
        case BOOT:              return "BOOT";
        case STANDBY:           return "STANDBY";
        case PRIMARY:           return "PRIMARY";
        case SINGLE_FC:         return "SINGLE_FC";
        case PARTIAL_PRIMARY:   return "PARTIAL_PRIMARY";
        case PARTIAL_SECONDARY: return "PARTIAL_SECONDARY";
        case CAN_INOP:          return "CAN_INOP";
        case ERROR:             return "ERROR";
        case SAFETY_DEGRADED:   return "SAFETY_DEGRADED";
        case STOP:              return "STOP";
        default:                return "UNKNOWN_STATE (" + std::to_string(state) + ")";
    }
}

void AegisBase::receivedMotor10(){
    motor10NodeTimer.restart();
    motor10NodeActive = true;
    onMotorNodeMessageReceived(10);
}

void AegisBase::receivedMotor11(){
    motor11NodeTimer.restart();
    motor11NodeActive = true;
    onMotorNodeMessageReceived(11);
}

void AegisBase::receivedMotor12(){
    motor12NodeTimer.restart();
    motor12NodeActive = true;
    onMotorNodeMessageReceived(12);
}

void AegisBase::receivedMotor13(){
    motor13NodeTimer.restart();
    motor13NodeActive = true;
    onMotorNodeMessageReceived(13);
}

void AegisBase::receivedMotor14(){
    motor14NodeTimer.restart();
    motor14NodeActive = true;
    onMotorNodeMessageReceived(14);
}

void AegisBase::receivedMotor15(){
    motor15NodeTimer.restart();
    motor15NodeActive = true;
    onMotorNodeMessageReceived(15);
}

void AegisBase::receivedMotor16(){
    motor16NodeTimer.restart();
    motor16NodeActive = true;
    onMotorNodeMessageReceived(16);
}

void AegisBase::receivedMotor17(){
    motor17NodeTimer.restart();
    motor17NodeActive = true;
    onMotorNodeMessageReceived(17);
}

void AegisBase::receivedLogic(){
    logicNodeTimer.restart();
    logicNodeActive = true;
}

void AegisBase::receivedAutonomy(){
    autonomyNodeTimer.restart();
    autonomyNodeActive = true;
}   

void AegisBase::receivedExcavation(){
    excavationNodeTimer.restart();
    excavationNodeActive = true;
}

void AegisBase::receivedStatusMonitor(){
    statusMonitorNodeTimer.restart();
    statusMonitorNodeActive = true;
}

void AegisBase::receivedVideoStream(){
    videoStreamNodeTimer.restart();
    videoStreamNodeActive = true;
}

void AegisBase::receivedZedTracking(){
    zedTrackingNodeTimer.restart();
    zedTrackingNodeActive = true;
}

void AegisBase::updateConnectionStatus(bool connected){
    connectedToClient = connected;
    if(this->update_sender_state){
        if(systemStatus_ref == PRIMARY || systemStatus_ref == PARTIAL_PRIMARY 
        || systemStatus_ref == SINGLE_FC || !remoteStatus.CONNECTED){
            this->update_sender_state(true);
        }
        else{
            this->update_sender_state(false);
        }
    }
    alertConnectionChange();
}

void AegisBase::checkNodeTimers(){
    if(motor10NodeTimer.isExpired()){
        motor10NodeActive = false;
    }
    if(motor11NodeTimer.isExpired()){
        motor11NodeActive = false;
    }
    
    if(motor12NodeTimer.isExpired()){
        motor12NodeActive = false;
    }
    
    if(motor13NodeTimer.isExpired()){
        motor13NodeActive = false;
    }
    
    if(motor14NodeTimer.isExpired()){
        motor14NodeActive = false;
    }
    
    if(motor15NodeTimer.isExpired()){
        motor15NodeActive = false;
    }
    
    if(motor16NodeTimer.isExpired()){
        motor16NodeActive = false;
    }
    
    if(motor17NodeTimer.isExpired()){
        motor17NodeActive = false;
    }
    
    if(logicNodeTimer.isExpired()){
        logicNodeActive = false;
    }
    
    if(autonomyNodeTimer.isExpired()){
        autonomyNodeActive = false;
    }
    
    if(excavationNodeTimer.isExpired()){
        excavationNodeActive = false;
    }
    
    if(statusMonitorNodeTimer.isExpired()){
        statusMonitorNodeActive = false;
    }
    
    if(videoStreamNodeTimer.isExpired()){
        videoStreamNodeActive = false;
    }
    
    if(zedTrackingNodeTimer.isExpired()){
        zedTrackingNodeActive = false;
    }
}

// ID 1
void AegisBase::sendSpeedMessage(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(1, "", 0);
}

// ID 2
void AegisBase::sendPositionMessage(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(2, "", 0);
}

// ID 10
void AegisBase::sendJoystickAxis(uint8_t which, uint8_t axis, float value) {
    if(!checkRemoteAlive()) return;
    JoystickAxis msg {which, axis, value};
    hb_link.send_data(10, &msg, sizeof(msg));
}

// ID 11
void AegisBase::sendJoystickButton(uint8_t which, uint8_t button, uint8_t state) {
    if(!checkRemoteAlive()) return;
    JoystickButton msg {which, button, state};
    hb_link.send_data(11, &msg, sizeof(msg));
}

// ID 12
void AegisBase::sendJoystickHat(uint8_t which, uint8_t hat, uint8_t value) {
    if(!checkRemoteAlive()) return;
    JoystickHat msg {which, hat, value};
    hb_link.send_data(12, &msg, sizeof(msg));
}

// ID 13
void AegisBase::sendKeyboardEvent(uint32_t keyval, uint8_t state) {
    if(!checkRemoteAlive()) return;
    KeyboardEvent msg {keyval, state};
    hb_link.send_data(13, &msg, sizeof(msg));
}

// ID 20
void AegisBase::sendBinaryMessage(BinaryMessage& binMsg) {
    if(!checkRemoteAlive()) return;
    auto bytesList = binMsg.getBytes();
    std::vector<uint8_t> buffer(bytesList->begin(), bytesList->end());
    hb_link.send_data(20, buffer.data(), buffer.size());
}

// --- 1xx Control Configuration ---
// ID 100
void AegisBase::sendAuth(){
    if(!checkRemoteAlive()) return;
    MotorAuthPayload payload;
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        payload.motor_states[i] = auth_table[i] ? 1 : 0;
    }
    hb_link.send_data(100, &payload, sizeof(payload));
}

// ID 101
void AegisBase::sendAuthConfirm(){
    MotorAuthPayload payload;
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        payload.motor_states[i] = auth_table[i] ? 1 : 0;
    }
    hb_link.send_data(101, &payload, sizeof(payload));
}

// ID 102 
void AegisBase::sendAuthRequest(){
    if(!checkRemoteAlive()) return;

    AuthRequestPayload payload{};
    // Request motors that: we can see on CAN, node is alive, 
    // remote has auth, we don't
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        bool can_visible = can0_table[i] || can1_table[i];
        bool node_alive = isMotorNodeAlive(static_cast<uint8_t>(i));
        if (can_visible && node_alive && remote_auth[i] && !auth_table[i]) {
            payload.motor_states[i] = 1;
        }
        else {
            payload.motor_states[i] = 0;
        }
    }
    payload.granted = 0; // Not used in request, set to 0

    hb_link.send_data(ID_REQ_AUTH, &payload, sizeof(payload));
}

// ID 103 
void AegisBase::sendAuthResponse(bool granted){
    if(!checkRemoteAlive()) return;

    AuthRequestPayload payload{};
    // Echo back which motors we're granting/denying
    // The actual motor_states are filled by the caller or set based on
    // what we're willing to release
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        if (granted) {
            // We're releasing motors we had auth for that the requester wants
            payload.motor_states[i] = auth_table[i] ? 1 : 0;
        } else {
            payload.motor_states[i] = 0;
        }
    }
    payload.granted = granted ? 1 : 0;

    hb_link.send_data(ID_AUTH_RESPONSE, &payload, sizeof(payload));
}
// --- 2xx State & Handshake ---
// ID 200
void AegisBase::queryControl(){
    if (!hb_link.is_remote_alive()){
        if(systemStatus_ref == STANDBY){
            requestStateTransition(SINGLE_FC);
            alertedRemoteMotors = false;
            alertedRemoteNodes = false;
            if(!motorsAuthorized)
                enableMotorAuthorization();
            alertSystemStatusChange();
            return;
        }
    }
    hb_link.send_data(200, "", 0);
}

// ID 201
void AegisBase::alertPrimary(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(201, "", 0);
}

// ID 202
void AegisBase::alertNotPrimary(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(202, "", 0);
}

// ID 203
void AegisBase::requestControl(){
    RCLCPP_INFO(nodeHandle->get_logger(), "Requesting control");
    if(!checkRemoteAlive()) return;
    RCLCPP_INFO(nodeHandle->get_logger(), "Sending request control");
    hb_link.send_data(203, "", 0);
}

// ID 204
void AegisBase::grantControl(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(204, "", 0);
}

// ID 205
void AegisBase::denyControl(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(205, "", 0);
}

// ID 206
void AegisBase::sendPing(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(206, "", 0);
}

// ID 207
void AegisBase::sendPong(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(207, "", 0);
}

// ID 208
void AegisBase::sendRelinquishRequest(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(208, "", 0);
}

// ID 209
void AegisBase::sendAcceptControl(){
    hb_link.send_data(209, "", 0);
}

// ID 210
void AegisBase::sendRejectControl(){
    hb_link.send_data(210, "", 0);
}

// ID 211
void AegisBase::alertSystemStatusChange(bool verbose){
    if(!checkRemoteAlive()) return;

    if (verbose) {
        RCLCPP_INFO(nodeHandle->get_logger(), "Sending SystemStatusChange: %d", (int)systemStatus_ref);
        std::cout << "SystemStatus: " << (int)systemStatus_ref << std::endl;
    }

    RemoteStatus status; 

    status.UP = true;
    status.WIFI_UP = wifi_up;
    status.CAN0_UP = can0_up;
    status.CAN1_UP = can1_up;
    status.CONNECTED = connectedToClient;
    status.STATUS = systemStatus_ref;

    hb_link.send_data(211, &status, sizeof(status));
}

// ID 212
void AegisBase::acknowledgeSystemStatusChange(bool error){
    uint8_t msg = (error) ? 1 : 0;
    hb_link.send_data(212, &msg, sizeof(msg));
}

// --- 3xx Parameter Exchange ---
// ID 300
void AegisBase::sendParamInit(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(300, "", 0);
}

// ID 301
void AegisBase::sendParamData(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(301, "", 0);
}

// ID 302
void AegisBase::sendParamAck(){
    hb_link.send_data(302, "", 0);
}

// ID 303
void AegisBase::sendParamReject(){
    hb_link.send_data(303, "", 0);
}

// ID 304
void AegisBase::sendSyncComplete(){
    hb_link.send_data(304, "", 0);
}

// ID 305
void AegisBase::sendReadyOp(){
    hb_link.send_data(305, "", 0);
}

// --- 4xx Operational Faults ---
// ID 400
void AegisBase::sendHardEStop(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(400, "", 0);
}

// ID 401
void AegisBase::sendSoftEStop(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(401, "", 0);
}


// ID 402
void AegisBase::alertLostMotor(uint8_t motor_id){
    if(!motorsAuthorized) enableMotorAuthorization();
    updateMotorAuthorization(motor_id, false);
    MotorListPayload msg; msg.count = 1; msg.motor_ids[0] = motor_id;
    checkMotorControlStatus();
    
    if(!checkRemoteAlive()) return;
    hb_link.send_data(402, &msg, sizeof(msg));
    sendAuth();
    if(systemStatus_ref == PRIMARY) requestStateTransition(PARTIAL_PRIMARY);
    alertSystemStatusChange();
}

// ID 403
void AegisBase::alertRegainedMotor(uint8_t motor_id){
    if(!motorsAuthorized) enableMotorAuthorization();
    updateMotorAuthorization(motor_id, true);
    MotorListPayload msg; msg.count = 1; msg.motor_ids[0] = motor_id;
    checkMotorControlStatus();
    
    if(!checkRemoteAlive()) return;
    hb_link.send_data(403, &msg, sizeof(msg));
    sendAuth();
    if(!checkRemoteAuthStatus() && systemStatus_ref == PARTIAL_PRIMARY) requestControl();
}

// ID 404
void AegisBase::alertWifiLost(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(404, "", 0);
}

// ID 405
void AegisBase::alertWifiRegained(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(405, "", 0);
}

// ID 406
void AegisBase::acknowledgeWifiChange(){
    hb_link.send_data(406, "", 0);
}

// ID 407


// ID 408


// ID 409


// ID 410


// ID 411


// ID 412


// ID 413


// ID 414


// ID 415


// ID 416


// ID 417


// ID 418


// ID 419


// ID 420


// ID 421


// ID 422
void AegisBase::alertMotorsDetected(){
    if(!checkRemoteAlive()) return;
    if(alertedRemoteMotors) return;

    MotorAuthPayload payload;
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        payload.motor_states[i] = can0_table[i] || can1_table[i];
    }
    hb_link.send_data(422, &payload, sizeof(payload));
    alertedRemoteMotors = true;
}

// ID 423
void AegisBase::acknowledgeMotorsDetected(){
    hb_link.send_data(423, "", 0);
    if(!alertedRemoteMotors){
        alertMotorsDetected();
    }
}

// ID 424
void AegisBase::alertNodesDetected(){
    if(!checkRemoteAlive()) return;
    if(alertedRemoteNodes) return;

    NodeAuthPayload payload;
    for (size_t i = 0; i < MAX_NODES; i++) {
        payload.node_states[i] = false;
    }
    hb_link.send_data(424, &payload, sizeof(payload));
    alertedRemoteNodes = true;
}

// ID 425
void AegisBase::acknowledgeNodesDetected(){
    hb_link.send_data(425, "", 0);
    if(!alertedRemoteNodes){
        alertNodesDetected();
    }
}

// ID 426
void AegisBase::alertLostNode(uint8_t node_lost){
    if(!checkRemoteAlive()) return;
    uint8_t msg = node_lost;
    hb_link.send_data(426, &msg, sizeof(msg));
}

// ID 427
void AegisBase::alertRegainedNode(uint8_t node_regained){
    if(!checkRemoteAlive()) return;
    uint8_t msg = node_regained;
    hb_link.send_data(427, &msg, sizeof(msg));
}

// ID 428
void AegisBase::acknowledgeNodeChange(){
    hb_link.send_data(428, "", 0);
}

void AegisBase::alertConnectionChange(){
    if(!checkRemoteAlive()) return;
    bool msg = connectedToClient;
    hb_link.send_data(429, &msg, sizeof(msg));
}

void AegisBase::acknowledgeConnectionChange(){
    hb_link.send_data(430, "", 0);
}

// --- 5xx System ---
// ID 500
void AegisBase::alertSystemShutdown(){
    hb_link.send_data(500, "", 0);
}

// ID 501
void AegisBase::alertSystemBoot(){
    hb_link.send_data(501, "", 0);
}

// ID 502
void AegisBase::alertSystemBootAck(){
    hb_link.send_data(502, "", 0);
}

void AegisBase::updateMotorAuthorization(uint8_t motor_id, bool authorized) {
    uint8_t adj_id;
    if(motor_id > MAX_MOTOR_ID)
        adj_id = motor_id - 10;
    else
        adj_id = motor_id;
    if (adj_id < MAX_MOTOR_ID) {
        auth_table[adj_id] = authorized;
        if (update_motor_auth) {
            update_motor_auth(adj_id, authorized);
        }
    }
    else {
        std::cout << "Motor ID out of bounds: " << (int)motor_id << std::endl;
    }
    std::cout << "Motor ID " << (int)adj_id << ": " << authorized << std::endl;
    checkMotorControlStatus();
}

void AegisBase::updateMotorCAN0State(uint8_t motor_id, bool up){
    uint8_t adj_id = motor_id - 10;
    if (adj_id < MAX_MOTOR_ID) {
        can0_table[adj_id] = up;
    }
    else {
        std::cout << "Motor ID out of bounds: " << motor_id << std::endl;
    }
}

void AegisBase::updateMotorCAN1State(uint8_t motor_id, bool up){
    uint8_t adj_id = motor_id - 10;
    if (adj_id < MAX_MOTOR_ID) {
        can1_table[adj_id] = up;
    }
    else {
        std::cout << "Motor ID out of bounds: " << motor_id << std::endl;
    }
}

bool AegisBase::isMotorDetectedCAN0(uint8_t motor_id){
    uint8_t adj_id = motor_id - 10;
    if (adj_id < MAX_MOTOR_ID) {
        return can0_table[adj_id];
    }
    return false;
}

bool AegisBase::isMotorDetectedCAN1(uint8_t motor_id){
    uint8_t adj_id = motor_id - 10;
    if (adj_id < MAX_MOTOR_ID) {
        return can1_table[adj_id];
    }
    return false;
}

bool AegisBase::isMotorAuthorized(uint8_t motor_id) const {
    uint8_t adj_id = motor_id - 10;
    if (adj_id < MAX_MOTOR_ID) {
        return auth_table[adj_id];
    }
    return false;
}

void AegisBase::enableMotorAuthorization(){
    if (handshakeStatus_ref == CONTROL_HANDSHAKE && handshakeStatus_ref == PARAM_HANDSHAKE) {
        std::cout << "[Auth] Deferring motor check (Handshake in progress)" << std::endl;
        return;
    }
    if(systemStatus_ref == PRIMARY || systemStatus_ref == PARTIAL_PRIMARY || 
       systemStatus_ref == SINGLE_FC || systemStatus_ref == PARTIAL_SECONDARY){
        std::cout << "Enabling motor authorization" << std::endl;
        for(int i = 0; i < MAX_MOTOR_ID; i++){
            if(can0_table[i] || can1_table[i]){
                auth_table[i] = true;
                if (update_motor_auth) update_motor_auth(i, true);
            }
            else{
                std::cout << "Motor " << i << " not authorized" << std::endl;
                auth_table[i] = false;
                if (update_motor_auth) update_motor_auth(i, false);
                if(systemStatus_ref == PRIMARY){
                    std::cout << "Switching to PARTIAL_PRIMARY" << std::endl;
                    requestStateTransition(PARTIAL_PRIMARY);
                    alertSystemStatusChange();
                }
                else if(systemStatus_ref == SINGLE_FC){
                    std::cout << "Switching to STOP" << std::endl;
                    requestStateTransition(STOP);
                    alertSystemStatusChange();
                }
                else if(systemStatus_ref == PARTIAL_PRIMARY){
                    std::cout << "Switching to STOP (from PARTIAL_PRIMARY)" << std::endl;
                    requestStateTransition(STOP);
                    alertSystemStatusChange();
                }
                else if(systemStatus_ref == PARTIAL_SECONDARY){
                    std::cout << "Switching to STOP (from PARTIAL_SECONDARY)" << std::endl;
                    requestStateTransition(STOP);
                    alertSystemStatusChange();
                }
            }
        }
        motorsAuthorized = true;
    }
}

bool AegisBase::processRemoteAuth(const uint8_t motor_states[MAX_MOTORS]){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        // Only one computer can have authorization for any motor
        if(auth_table[i] == motor_states[i] && auth_table[i] == 1){
            auth_table[i] = !motor_states[i];
            if (update_motor_auth) update_motor_auth(i, false);
            std::cout << "MOTOR " << i + 10 << " not authorized" << std::endl;
        }
        remote_auth[i] = motor_states[i];
        if(remote_auth[i])
            std::cout << "REMOTE MOTOR " << i + 10 << " authorized" << std::endl;
        else
            std::cout << "REMOTE MOTOR " << i + 10 << " not authorized" << std::endl;
    }
    return true;
}

void AegisBase::setAuthFromRemote(const uint8_t motor_states[MAX_MOTORS]){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        if(!motor_states[i]){
            if(can0_table[i] || can1_table[i]){
                auth_table[i] = true;
                if (update_motor_auth) update_motor_auth(i, true);
                std::cout << "MOTOR " << i + 10 << " authorized" << std::endl;
            }
            else{
                auth_table[i] = false;
                if (update_motor_auth) update_motor_auth(i, false);
                std::cout << "MOTOR " << i + 10 << " not authorized" << std::endl;
            }
        }
        else{
            auth_table[i] = false;
            if (update_motor_auth) update_motor_auth(i, false);
            std::cout << "MOTOR " << i + 10 << " not authorized" << std::endl;

        }
    }
}

bool AegisBase::checkAuthErrors(){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        if(auth_table[i] == remote_auth[i]){
            if(auth_table[i] == 1){
                std::cout << "ERROR: Duplicate auth." << std::endl;
            }
            else{
                std::cout << "ERROR: Both motors failed to auth" << std::endl;
            }
            return true;
        }
    }
    return false;
}

// Used to check whether or not the FC has any authorization, need to rename
bool AegisBase::checkAuthStatus(){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        if(auth_table[i]){
            return true;
        }
    }
    return false;
}

// Used to check whether or not the peer FC has any authorization, need to rename
bool AegisBase::checkRemoteAuthStatus(){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        if(remote_auth[i]){
            RCLCPP_INFO(nodeHandle->get_logger(), "remote_auth[%d] true", i);
            return true;
        }
    }
    return false;
}

void AegisBase::processRemoteControl(const uint8_t motor_states[MAX_MOTORS]){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        remote_cont[i] = motor_states[i];
    }
}


void AegisBase::processLostMotor(const uint8_t motor_states[MAX_MOTORS]){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        remote_cont[i] = motor_states[i];
    }
}

void AegisBase::processRegainedMotor(const uint8_t motor_states[MAX_MOTORS]){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        remote_cont[i] = motor_states[i];
    }
}

void AegisBase::processLostNode(uint8_t node){

}

void AegisBase::processRegainedNode(uint8_t node){

}

bool AegisBase::checkAllMotorsInit(){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        if(!(can0_table[i] || can1_table[i])){
            return true;
        }
    }
    return false;
}

bool AegisBase::canGiveControl(){
    // TODO: Add checks to determine whether the system is in a state that it can
    // give control back to the other FC. This will most likely include a check on
    // whether or not the motors are moving, whether the other FC can control all 
    // motors and any other checks necessary to determine system safety.
    return true;
}

bool AegisBase::inControl(){
    if(systemStatus_ref == PRIMARY || systemStatus_ref == PARTIAL_PRIMARY){
        return true;
    }
    return false;
}

bool AegisBase::canAcceptControl(){
    return true;
}

bool AegisBase::checkControlErrors(){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        if(!(remote_cont[i] || can0_table[i] || can1_table[i])){
            return true;
        }
    }
    return false;
}

void AegisBase::checkMotorControlStatus(){
    bool all_motors_ok = true;
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        if((!can0_table[i] && !can1_table[i]) || !auth_table[i]){
            all_motors_ok = false;
            if(systemStatus_ref == SINGLE_FC){
                std::cout << "Entering Stop state" << std::endl;
                requestStateTransition(STOP);
                return;
            }
            // Don't return early for other states — we need to check
            // if we should stay in current state
        }
    }

    if(all_motors_ok){
        if(systemStatus_ref == STOP){
            if(!remoteStatus.UP){
                // No peer — go to SINGLE_FC
                requestStateTransition(SINGLE_FC);
                alertedRemoteMotors = false;
                alertedRemoteNodes = false;
                if(!motorsAuthorized)
                    enableMotorAuthorization();
            }
            else{
                // Peer is alive — go to PARTIAL_PRIMARY
                // (will transition to PRIMARY after Nano grants control)
                requestStateTransition(PARTIAL_PRIMARY);
                alertSystemStatusChange();
            }
        }
        else if(systemStatus_ref == PARTIAL_PRIMARY){
            // All motors recovered while in partial state
            // Request takeover if remote has no auth
            if(!checkRemoteAuthStatus()){
                requestControl();
            }
        }
    }
}
void AegisBase::applyRemoteAlivePolicy() {
    bool alive = hb_link.is_remote_alive();
    
    remoteStatus.UP = alive;

    if (alive) {
        clearedRemoteAuth = false;
    }
    else {
        if (systemStatus_ref == STOP) {
            if (!clearedRemoteAuth) {
                std::cout << "[Watchdog] Remote Dead. Removing remote authorization" << std::endl;
                for (size_t i = 0; i < MAX_MOTORS; i++) {
                    remote_auth[i] = 0;
                }
                alertedRemoteMotors = false;
                alertedRemoteNodes = false;
                clearedRemoteAuth = true; 
            }
            return;
        }
        if (systemStatus_ref != BOOT && 
            systemStatus_ref != ERROR && systemStatus_ref != SINGLE_FC) {
            
            std::cout << "[Watchdog] Remote Dead. Transitioning to SINGLE_FC." << std::endl;
            handshakeStatus_ref = IDLE_HANDSHAKE;
            RCLCPP_INFO(nodeHandle->get_logger(), "Removing remote authorization");
            for (size_t i = 0; i < MAX_MOTORS; i++) {
                remote_auth[i] = 0;
            }
            requestStateTransition(SINGLE_FC);
            alertedRemoteMotors = false;
            alertedRemoteNodes = false;
            if(!motorsAuthorized)
                enableMotorAuthorization();
        }
    }
}

bool AegisBase::checkRemoteAlive(){
    if (remote_shutdown_latched.load()) return false;
    return hb_link.is_remote_alive();
}

bool AegisBase::isHandshakeMsg(uint16_t id){
    if(id == 100 || id == 101 ||
       id == 200 || id == 201 || id == 202 || id == 211 || id == 212 ||
       id >= 300 && id <= 305 || 
       id == 422 || id == 423 || id == 424 || id == 425)
        return true;
    return false;
}

bool AegisBase::isMotorNodeAlive(uint8_t motor_index) const {
    switch (motor_index) {
        case 0: return motor10NodeActive;
        case 1: return motor11NodeActive;
        case 2: return motor12NodeActive;
        case 3: return motor13NodeActive;
        case 4: return motor14NodeActive;
        case 5: return motor15NodeActive;
        case 6: return motor16NodeActive;
        case 7: return motor17NodeActive;
        default: return false;
    }
}

void AegisBase::processStatusMonitorCANReport(const uint8_t can0_states[MAX_MOTORS],
                                               const uint8_t can1_states[MAX_MOTORS]) 
{
    bool any_auth_changed = false;

    for (size_t i = 0; i < MAX_MOTORS; i++) {
        bool old_can0 = can0_table[i];
        bool old_can1 = can1_table[i];
        bool new_can0 = can0_states[i] != 0;
        bool new_can1 = can1_states[i] != 0;

        can0_table[i] = new_can0;
        can1_table[i] = new_can1;

        bool was_visible = old_can0 || old_can1;
        bool now_visible = new_can0 || new_can1;

        if (!was_visible && now_visible) {
            std::cout << "[StatusMonitor] Motor " << (i + 10) 
                      << " now visible on CAN." << std::endl;

            if (systemStatus_ref == PRIMARY || systemStatus_ref == PARTIAL_PRIMARY || 
                systemStatus_ref == SINGLE_FC || systemStatus_ref == STOP) 
            {
                if (evaluateAndSelfAuthorize(static_cast<uint8_t>(i))) {
                    any_auth_changed = true;
                }
            }
        } 
        else if (was_visible && !now_visible) {
            std::cout << "[StatusMonitor] Motor " << (i + 10) 
                      << " lost from CAN." << std::endl;
            alertLostMotor(static_cast<uint8_t>(i + 10));
        }
    }

    if (any_auth_changed) {
        if (checkRemoteAlive()) {
            sendAuth();
        }
        checkMotorControlStatus();
    }
}

void AegisBase::onMotorNodeMessageReceived(uint8_t motor_id) {
    uint8_t idx;
    if (motor_id >= 10) {
        idx = motor_id - 10;
    } else {
        idx = motor_id;
    }

    if (idx >= MAX_MOTOR_ID) {
        std::cerr << "[Auth] onMotorNodeMessageReceived: motor_id " 
                  << (int)motor_id << " out of range." << std::endl;
        return;
    }

    // Only attempt self-authorization if we're in a controlling state
    if (systemStatus_ref != PRIMARY && systemStatus_ref != PARTIAL_PRIMARY && 
        systemStatus_ref != SINGLE_FC && systemStatus_ref != STOP) {
        return;
    }

    // If we're already authorized, nothing to do
    if (auth_table[idx]) return;

    // Check if CAN is visible for this motor
    bool can_visible = can0_table[idx] || can1_table[idx];
    if (!can_visible) return;

    if (!remote_auth[idx]) {
        // Nobody owns it — claim it
        if (evaluateAndSelfAuthorize(idx)) {
            if (checkRemoteAlive()) {
                sendAuth();
            }
            // Check if this authorization completes the set
            checkMotorControlStatus();
        }
    }
    else {
        // Remote owns it — need to request it via 102
        std::cout << "[Auth] Motor " << (int)(idx + 10) 
                  << " node alive and CAN visible, but remote has auth. Sending 102."
                  << std::endl;
        sendAuthRequest();
    }
}

bool AegisBase::evaluateAndSelfAuthorize(uint8_t motor_index) {
    if (motor_index >= MAX_MOTOR_ID) return false;

    // Already authorized
    if (auth_table[motor_index]) return false;

    // Must be visible on at least one CAN bus
    bool can_visible = can0_table[motor_index] || can1_table[motor_index];
    if (!can_visible) return false;

    // Node must be alive
    if (!isMotorNodeAlive(motor_index)) return false;

    // Remote must NOT have it
    if (remote_auth[motor_index]) return false;

    // All conditions met
    auth_table[motor_index] = true;
    std::cout << "[Auth] Self-authorized motor " << (int)(motor_index + 10) 
              << " (CAN visible, node alive, remote not authorized)." << std::endl;

    return true;
}