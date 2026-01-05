#include "AegisBase.hpp"
void AegisBase::sendJoystickAxis(uint8_t which, uint8_t axis, float value) {
    if (!hb_link.is_remote_alive()) return;
    JoystickAxis msg {which, axis, value};
    hb_link.send_data(010, &msg, sizeof(msg));
}


void AegisBase::sendJoystickButton(uint8_t which, uint8_t button, uint8_t state) {
    if (!hb_link.is_remote_alive()) return;
    JoystickButton msg {which, button, state};
    hb_link.send_data(011, &msg, sizeof(msg));
}


void AegisBase::sendJoystickHat(uint8_t which, uint8_t hat, uint8_t value) {
    if (!hb_link.is_remote_alive()) return;
    JoystickHat msg {which, hat, value};
    hb_link.send_data(012, &msg, sizeof(msg));
}


void AegisBase::sendKeyboardEvent(uint32_t keyval, uint8_t state) {
    if (!hb_link.is_remote_alive()) return;
    KeyboardEvent msg {keyval, state};
    hb_link.send_data(013, &msg, sizeof(msg));
}


void AegisBase::sendBinaryMessage(BinaryMessage& binMsg) {
    if (!hb_link.is_remote_alive()) return;
    auto bytesList = binMsg.getBytes();
    std::vector<uint8_t> buffer(bytesList->begin(), bytesList->end());
    hb_link.send_data(020, buffer.data(), buffer.size());
}


void AegisBase::queryControl(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(200, "", 0);
}


void AegisBase::alertPrimary(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(201, "", 0);
}


void AegisBase::alertNotPrimary(){
    if (!hb_link.is_remote_alive()) return;
    hb_link.send_data(202, "", 0);
}


void AegisBase::alertSystemStatusChange(){
    if (!hb_link.is_remote_alive()) return;
    RCLCPP_INFO(nodeHandle->get_logger(), "Sending SystemStatusChange");
    uint8_t msg = systemStatus_ref;
    hb_link.send_data(211, &msg, sizeof(msg));
}


void AegisBase::acknowledgeSystemStatusChange(bool error){
    if (!hb_link.is_remote_alive()) return;
    uint8_t msg = (error) ? 1 : 0;
    hb_link.send_data(212, &msg, sizeof(msg));
}
