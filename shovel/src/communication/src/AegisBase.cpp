#include "AegisBase.hpp"

// AegisBase.cpp

void AegisBase::initAegis(){
    systemStatus_ref = BOOT;    
    std::cout << "[System] Booting... Waiting 1s for peripherals." << std::endl;
    boot_timer.start(1000); 
}

// You likely need to add this logic to a check function called by your main loop
// Since AegisBase doesn't have a main loop, ensure your Controller's checkTimers calls this logic.
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

    if (!isValidTransition(systemStatus_ref, new_state)) {
        std::cerr << "[FSM] ILLEGAL TRANSITION ATTEMPT: " 
                  << stateToString(systemStatus_ref) << " -> " 
                  << stateToString(new_state) << std::endl;
        return;
    }

    std::cout << "[FSM] Transition: " << stateToString(systemStatus_ref) 
              << " -> " << stateToString(new_state) << std::endl;

    onExitState(systemStatus_ref);
    systemStatus_ref = new_state;
    onEnterState(new_state);
    alertSystemStatusChange();
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
                    to == PARTIAL_SECONDARY);
            
        case STOP:
            return (to == PARTIAL_PRIMARY || // Recovered FC2, still missing motor
                    to == SINGLE_FC ||       // Recovered motor, still missing FC2
                    to == ERROR);            // Gave up
            
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
    if(!checkRemoteAlive()) return;
    MotorAuthPayload payload;
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        payload.motor_states[i] = auth_table[i] ? 1 : 0;
    }
    hb_link.send_data(101, &payload, sizeof(payload));
}

// --- 2xx State & Handshake ---
// ID 200
void AegisBase::queryControl(){
    if (!hb_link.is_remote_alive()){
        if(systemStatus_ref == STANDBY){
            requestStateTransition(SINGLE_FC);
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
    if(!checkRemoteAlive()) return;
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
    if(!checkRemoteAlive()) return;
    hb_link.send_data(209, "", 0);
}

// ID 210
void AegisBase::sendRejectControl(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(210, "", 0);
}

// ID 211
void AegisBase::alertSystemStatusChange(){
    if(!checkRemoteAlive()) return;
    RCLCPP_INFO(nodeHandle->get_logger(), "Sending SystemStatusChange");
    std::cout << "SystemStatus: " << (int)systemStatus_ref << std::endl;
    uint8_t msg = systemStatus_ref;
    hb_link.send_data(211, &msg, sizeof(msg));
}

// ID 212
void AegisBase::acknowledgeSystemStatusChange(bool error){
    if(!checkRemoteAlive()) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Skipping 212 ACK: remote not alive");
        return;
    }
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
    if(!checkRemoteAlive()) return;
    hb_link.send_data(302, "", 0);
}

// ID 303
void AegisBase::sendParamReject(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(303, "", 0);
}

// ID 304
void AegisBase::sendSyncComplete(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(304, "", 0);
}

// ID 305
void AegisBase::sendReadyOp(){
    if(!checkRemoteAlive()) return;
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
    if(!motorsAuthorized)
        enableMotorAuthorization();
    updateMotorAuthorization(motor_id, false);
    MotorListPayload msg;
    msg.count = 1;
    msg.motor_ids[0] = motor_id;
    checkMotorControlStatus();
    if(!checkRemoteAlive()) return;
    hb_link.send_data(402, &msg, sizeof(msg));
    sendAuth();
    if(systemStatus_ref == PRIMARY){
        requestStateTransition(PARTIAL_PRIMARY);
    }
    alertSystemStatusChange();
}

// ID 403
void AegisBase::alertRegainedMotor(uint8_t motor_id){
    if(!motorsAuthorized)
        enableMotorAuthorization();
    updateMotorAuthorization(motor_id, true);
    MotorListPayload msg;
    msg.count = 1;
    msg.motor_ids[0] = motor_id;
    checkMotorControlStatus();
    if(!checkRemoteAlive()) return;
    hb_link.send_data(403, &msg, sizeof(msg));
    sendAuth();
    if(!checkRemoteAuthStatus()){
        if(systemStatus_ref == PARTIAL_PRIMARY){
            requestControl();
        }
    }
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
    if(!checkRemoteAlive()) return;
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
    
    MotorAuthPayload payload;
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        payload.motor_states[i] = can0_table[i] || can1_table[i];
    }
    hb_link.send_data(422, &payload, sizeof(payload));
    alertedRemoteMotors = true;
}

// ID 423
void AegisBase::acknowledgeMotorsDetected(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(423, "", 0);
    if(!alertedRemoteMotors){
        alertMotorsDetected();
    }
}

// ID 424
void AegisBase::alertLostNode(uint8_t node_lost){
    if(!checkRemoteAlive()) return;
    uint8_t msg = node_lost;
    hb_link.send_data(424, &msg, sizeof(msg));
}

// ID 425
void AegisBase::alertRegainedNode(uint8_t node_regained){
    if(!checkRemoteAlive()) return;
    uint8_t msg = node_regained;
    hb_link.send_data(425, &node_regained, sizeof(node_regained));
}

// ID 426
void AegisBase::acknowledgeNodeChange(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(426, "", 0);
}

// --- 5xx System ---
// ID 500
void AegisBase::alertSystemShutdown(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(500, "", 0);
}

// ID 501
void AegisBase::alertSystemBoot(){
    if(!checkRemoteAlive()) return;
    hb_link.send_data(501, "", 0);
}

// ID 502
void AegisBase::alertSystemBootAck(){
    if(!checkRemoteAlive()) return;
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
    if(systemStatus_ref == PRIMARY || systemStatus_ref == PARTIAL_PRIMARY || systemStatus_ref == SINGLE_FC){
        std::cout << "Enabling motor authorization" << std::endl;
        for(int i = 0; i < MAX_MOTOR_ID; i++){
            if(can0_table[i] || can1_table[i]){
                auth_table[i] = true;
            }
            else{
                std::cout << "Motor " << i << "not authorized" << std::endl;
                auth_table[i] = false;
                if(systemStatus_ref == PRIMARY){
                    std::cout << "Switching to PARTIAL_PRIMARY" << std::endl;
                    requestStateTransition(PARTIAL_PRIMARY);
                    alertSystemStatusChange();
                }
            }
        }
        motorsAuthorized = true;
    }
}

bool AegisBase::processRemoteAuth(const uint8_t motor_states[MAX_MOTORS]){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        if(auth_table[i] == motor_states[i] && auth_table[i] == 1){
            auth_table[i] = !motor_states[i];
        }
        remote_auth[i] = motor_states[i];
    }
    return true;
}

void AegisBase::setAuthFromRemote(const uint8_t motor_states[MAX_MOTORS]){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        if(!motor_states[i]){
            if(can0_table[i] || can1_table[i]){
                auth_table[i] = true;
            }
        }
        else{
            auth_table[i] = false;
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
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        std::cout << "!can0_table[" << i << "]: " << !can0_table[i] << "!can1_table[i]: " << !can1_table[i] << 
        "!auth_table[i]: " << !auth_table[i] << std::endl;
        if((!can0_table[i] && !can1_table[i]) || !auth_table[i]){
            std::cout << "Here" << std::endl;
            if(systemStatus_ref == SINGLE_FC){
                std::cout << "Entering Stop state" << std::endl;
                requestStateTransition(STOP);
            }
            return;
        }
    }
    if(systemStatus_ref == STOP){
        if(remoteStatus.UP == false){
            requestStateTransition(SINGLE_FC);
            if(!motorsAuthorized)
                enableMotorAuthorization();
        }
    }
}

bool AegisBase::checkRemoteAlive(){
    if (hb_link.is_remote_alive()) {
        remoteStatus.UP = true;
        return true;
    }
    remoteStatus.UP = false;
    if (systemStatus_ref != BOOT && systemStatus_ref != STOP && systemStatus_ref != ERROR) {
        requestStateTransition(SINGLE_FC);
        if(!motorsAuthorized)
            enableMotorAuthorization();
    }
    return false;
}

bool AegisBase::isHandshakeMsg(uint16_t id){
    if(id == 100 ||
       id == 101 ||
       id == 200 ||
       id == 201 ||
       id == 202 ||
       id == 211 ||
       id == 212 ||
       id == 300 || 
       id == 301 || 
       id == 302 || 
       id == 303 || 
       id == 304 || 
       id == 305 || 
       id == 422 || 
       id == 423)
        return true;
    return false;
}