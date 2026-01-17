#include "AegisBase.hpp"
// ID 1

// ID 2

// ID 10
void AegisBase::sendJoystickAxis(uint8_t which, uint8_t axis, float value) {
    if (!hb_link.is_remote_alive()) return;
    JoystickAxis msg {which, axis, value};
    hb_link.send_data(010, &msg, sizeof(msg));
}

// ID 11
void AegisBase::sendJoystickButton(uint8_t which, uint8_t button, uint8_t state) {
    if (!hb_link.is_remote_alive()) return;
    JoystickButton msg {which, button, state};
    hb_link.send_data(011, &msg, sizeof(msg));
}

// ID 12
void AegisBase::sendJoystickHat(uint8_t which, uint8_t hat, uint8_t value) {
    if (!hb_link.is_remote_alive()) return;
    JoystickHat msg {which, hat, value};
    hb_link.send_data(012, &msg, sizeof(msg));
}

// ID 13
void AegisBase::sendKeyboardEvent(uint32_t keyval, uint8_t state) {
    if (!hb_link.is_remote_alive()) return;
    KeyboardEvent msg {keyval, state};
    hb_link.send_data(013, &msg, sizeof(msg));
}

// ID 20
void AegisBase::sendBinaryMessage(BinaryMessage& binMsg) {
    if (!hb_link.is_remote_alive()) return;
    auto bytesList = binMsg.getBytes();
    std::vector<uint8_t> buffer(bytesList->begin(), bytesList->end());
    hb_link.send_data(020, buffer.data(), buffer.size());
}

// --- 1xx Control Configuration ---
// ID 100
void AegisBase::sendAuth(){
    if (!hb_link.is_remote_alive()) return;
    MotorAuthPayload payload;
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        payload.motor_states[i] = auth_table[i] ? 1 : 0;
    }
    hb_link.send_data(100, &payload, sizeof(payload));
}

// ID 101
void AegisBase::sendAuthConfirm(){
    if (!hb_link.is_remote_alive()) return;
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
        systemStatus_ref = SINGLE_FC;
        alertSystemStatusChange();
        return;
    }
    hb_link.send_data(200, "", 0);
}

// ID 201
void AegisBase::alertPrimary(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(201, "", 0);
}

// ID 202
void AegisBase::alertNotPrimary(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(202, "", 0);
}

// ID 203
void AegisBase::requestControl(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(203, "", 0);
}

// ID 204
void AegisBase::grantControl(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(204, "", 0);
}

// ID 205
void AegisBase::denyControl(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(205, "", 0);
}

// ID 206
void AegisBase::sendPing(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(206, "", 0);
}

// ID 207
void AegisBase::sendPong(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(206, "", 0);
}

// ID 208


// ID 209


// ID 210


// ID 211
void AegisBase::alertSystemStatusChange(){
    if (!hb_link.is_remote_alive()) return;
    RCLCPP_INFO(nodeHandle->get_logger(), "Sending SystemStatusChange");
    uint8_t msg = systemStatus_ref;
    hb_link.send_data(211, &msg, sizeof(msg));
}

// ID 212
void AegisBase::acknowledgeSystemStatusChange(bool error){
    if (!hb_link.is_remote_alive()) return;
    uint8_t msg = (error) ? 1 : 0;
    hb_link.send_data(212, &msg, sizeof(msg));
}

// --- 3xx Parameter Exchange ---
// ID 300


// ID 301


// ID 302


// ID 303


// ID 304


// ID 305


// --- 4xx Operational Faults ---
// ID 400


// ID 401


// ID 402
void AegisBase::alertLostMotor(uint8_t motor_id){
    updateMotorAuthorization(motor_id, false);
    MotorListPayload msg;
    msg.count = 1;
    msg.motor_ids[0] = motor_id;
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(402, &msg, sizeof(msg));
    sendAuth();
    if(systemStatus_ref == PRIMARY){
        systemStatus_ref = PARTIAL_PRIMARY;
    }
    alertSystemStatusChange();
}

// ID 403
void AegisBase::alertRegainedMotor(uint8_t motor_id){
    updateMotorAuthorization(motor_id, true);
    MotorListPayload msg;
    msg.count = 1;
    msg.motor_ids[0] = motor_id;
    if (!hb_link.is_remote_alive()) return;
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
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(404, "", 0);
}

// ID 405
void AegisBase::alertWifiRegained(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(405, "", 0);
}

// ID 406
void AegisBase::acknowledgeWifiChange(){
    if (!hb_link.is_remote_alive()) return;
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
    if (!hb_link.is_remote_alive()) return;
    
    MotorAuthPayload payload;
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        payload.motor_states[i] = can0_table[i] || can1_table[i];
    }
    hb_link.send_data(422, &payload, sizeof(payload));
    alertedRemoteMotors = true;
}

// ID 423
void AegisBase::acknowledgeMotorsDetected(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(423, "", 0);
    if(!alertedRemoteMotors){
        alertMotorsDetected();
    }
}


// --- 5xx System ---
// ID 500
void AegisBase::alertSystemShutdown(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(500, "", 0);
}

// ID 501
void AegisBase::alertSystemBoot(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(501, "", 0);
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
    for(int i = 0; i < MAX_MOTOR_ID; i++){
        if(can0_table[i] || can1_table[i]){
            auth_table[i] = true;
        }
        else{
            auth_table[i] = false;
            if(systemStatus_ref == PRIMARY){
                std::cout << "Switching to PARTIAL_PRIMARY" << std::endl;
                systemStatus_ref = PARTIAL_PRIMARY;
                alertSystemStatusChange();
            }
        }
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

bool AegisBase::checkAuth(){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        if(auth_table[i] == remote_auth[i]){
            std::cout << "ERROR: Duplicate auth." << std::endl;
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

}


bool AegisBase::checkAllMotorsInit(){
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        if(!(can0_table[i] || can1_table[i])){
            return true;
        }
    }
    return false;
}

void AegisBase::checkMotorInitTimer() {
    if (!init_timer_active) return;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - init_start_time).count();

    if (elapsed >= 200) {
        init_timer_active = false;
        alertMotorsDetected();
    }
}