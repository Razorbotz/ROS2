#include <BinaryMessage.hpp>

void update_if_changed(BinaryMessage& msg, bool& changed, uint8_t& old_val, uint8_t new_val, const std::string& label) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementUInt8(label, new_val);
        old_val = new_val;
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, std::string& old_val, const std::string& new_val, const std::string& label) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementString(label, new_val);
        old_val = new_val;
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, uint16_t& old_val, uint16_t new_val, const std::string& label) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementUInt16(label, new_val);
        old_val = new_val;
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, float& old_val, float new_val, const std::string& label) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementFloat32(label, new_val);
        old_val = new_val;
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, bool& old_val, bool new_val, const std::string& label) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementBoolean(label, new_val);
        old_val = new_val;
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, int& old_val, int new_val, const std::string& label) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementInt32(label, new_val);
        old_val = new_val;
    }
}

//Extra for new FieldString type
void update_if_changed(BinaryMessage& msg, bool& changed, uint8_t& old_val, uint8_t new_val, Field_Strings field) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementUInt8(field, new_val);
        old_val = new_val;
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, std::string& old_val, const std::string& new_val, Field_Strings field) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementString(field, new_val);
        old_val = new_val;
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, uint16_t& old_val, uint16_t new_val, Field_Strings field) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementUInt16(field, new_val);
        old_val = new_val;
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, float& old_val, float new_val, const Field_Strings field) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementFloat32(field, new_val);
        old_val = new_val;
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, bool& old_val, bool new_val, Field_Strings field) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementBoolean(field, new_val);
        old_val = new_val;
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, int& old_val, int new_val, Field_Strings field) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementInt32(field, new_val);
        old_val = new_val;
    }
}