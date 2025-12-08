#include "ConfigDefinitions.hpp"

// --- Helper for Initialization ---
static void initialize_bool_map(std::map<std::string, bool>& map, const std::vector<std::string>& keys) {
    for (const auto& key : keys)
        map[key] = true;
}

// --- Generic Macro to Declare Accessors ---
#define DEFINE_KEY_GROUP(NAME, ...) \
std::vector<std::string>& get_##NAME##_keys() { \
    static std::vector<std::string> keys = { __VA_ARGS__ }; \
    return keys; \
} \
std::vector<std::string>& get_reset_##NAME##_keys() { \
    auto& keys = get_##NAME##_keys(); \
    keys = { __VA_ARGS__ }; \
    return keys; \
} \
std::map<std::string, bool>& get_##NAME##_values() { \
    static std::map<std::string, bool> values; \
    if (values.empty()) { \
        for (const auto& key : get_##NAME##_keys()) \
            values[key] = true; \
    } \
    return values; \
}

// --- Accessor Implementations ---
std::string& get_configFile() {
    static std::string configFile = "config.txt";
    return configFile;
}

std::set<std::string>& get_speedometer_keys() {
    static std::set<std::string> speedometer_keys = {
        "DISPLAY_SPEED", "NUMBERS_INSIDE", "NUMBER_TICKS"
    };
    return speedometer_keys;
}

// --- Subsystem Definitions ---
DEFINE_KEY_GROUP(talon,
    "Device ID", "Bus Voltage", "Output Current", "Output Percent",
    "Temperature", "Sensor Position", "Sensor Velocity", "Max Current"
)

DEFINE_KEY_GROUP(falcon, get_talon_keys())

DEFINE_KEY_GROUP(linear,
    "Motor Number", "Speed", "Potentiometer", "Time Without Change",
    "Max", "Min", "Error", "At Min", "At Max", "Distance", "Sensorless"
)

DEFINE_KEY_GROUP(power,
    "Voltage", "Temp", "Current 0", "Current 1", "Current 2",
    "Current 3", "Current 4", "Current 5", "Current 6"
)

DEFINE_KEY_GROUP(power2,
    "Current 7", "Current 8", "Current 9", "Current 10",
    "Current 11", "Current 12", "Current 13", "Current 14", "Current 15"
)

DEFINE_KEY_GROUP(autonomy,
    "Robot State", "Excavation State", "Error State", "Diagnostics State",
    "Tilt State", "Dump State", "Level Bucket", "Level Arms", "Dest X", "Dest Z"
)

DEFINE_KEY_GROUP(zed,
    "X", "Y", "Z", "roll", "pitch", "yaw", "aruco"
)

DEFINE_KEY_GROUP(communication,
    "RSSI", "Wi-Fi", "CAN Bus", "Using CAN1", "RX packets", "TX packets",
    "CAN Bus2", "RX2 packets", "TX2 packets", "Status"
)

DEFINE_KEY_GROUP(drivetrain,
    "F1 Vel", "F1 RPM", "F1 Speed",
    "F2 Vel", "F2 RPM", "F2 Speed",
    "F3 Vel", "F3 RPM", "F3 Speed",
    "F4 Vel", "F4 RPM", "F4 Speed"
)

// --- Central Maps ---
std::map<std::string, std::vector<std::string>*>& get_key_vectors() {
    static std::map<std::string, std::vector<std::string>*> key_vectors = {
        {"Talon", &get_talon_keys()},
        {"Falcon", &get_falcon_keys()},
        {"Linear", &get_linear_keys()},
        {"Autonomy", &get_autonomy_keys()},
        {"Communication", &get_communication_keys()},
        {"Power2", &get_power2_keys()},
        {"Power", &get_power_keys()},
        {"Zed", &get_zed_keys()},
        {"Drivetrain", &get_drivetrain_keys()}
    };
    return key_vectors;
}

std::map<std::string, std::vector<ElementInfo>>& get_element_definitions() {
    static std::map<std::string, std::vector<ElementInfo>> element_definitions = {
        {"TALON", {
            {ElementType::UInt8, "Device ID"}, {ElementType::UInt16, "Bus Voltage"},
            {ElementType::UInt16, "Output Current"}, {ElementType::Float32, "Output Percent"},
            {ElementType::Float32, "Sensor Velocity"}, {ElementType::UInt8, "Temperature"},
            {ElementType::UInt16, "Sensor Position"}, {ElementType::Float32, "Max Current"}
        }},
        {"FALCON", {
            {ElementType::UInt8, "Device ID"}, {ElementType::UInt16, "Bus Voltage"},
            {ElementType::UInt16, "Output Current"}, {ElementType::Float32, "Output Percent"},
            {ElementType::UInt8, "Temperature"}, {ElementType::Float32, "Sensor Position"},
            {ElementType::Float32, "Sensor Velocity"}, {ElementType::Float32, "Max Current"}
        }},
        {"LINEAR", {
            {ElementType::UInt8, "Motor Number"}, {ElementType::Float32, "Speed"},
            {ElementType::UInt16, "Potentiometer"}, {ElementType::UInt8, "Time Without Change"},
            {ElementType::UInt16, "Max"}, {ElementType::UInt16, "Min"},
            {ElementType::String, "Error"}, {ElementType::Boolean, "At Min"},
            {ElementType::Boolean, "At Max"}, {ElementType::Float32, "Distance"},
            {ElementType::Boolean, "Sensorless"}
        }},
        {"AUTONOMY", {
            {ElementType::String, "Robot State"}, {ElementType::String, "Excavation State"},
            {ElementType::String, "Error State"}, {ElementType::String, "Diagnostics State"},
            {ElementType::String, "Tilt State"}, {ElementType::String, "Dump State"},
            {ElementType::String, "Level Bucket"}, {ElementType::String, "Level Arms"},
            {ElementType::Float32, "Dest X"}, {ElementType::Float32, "Dest Z"}
        }},
        {"ZED", {
            {ElementType::Float32, "X"}, {ElementType::Float32, "Y"}, {ElementType::Float32, "Z"},
            {ElementType::Float32, "roll"}, {ElementType::Float32, "pitch"}, {ElementType::Float32, "yaw"},
            {ElementType::Boolean, "aruco"}
        }},
        {"COMMUNICATION", {
            {ElementType::Int32, "RSSI"}, {ElementType::String, "Wi-Fi"}, {ElementType::String, "CAN Bus"},
            {ElementType::Boolean, "Using CAN1"}, {ElementType::Int32, "RX packets"}, {ElementType::Int32, "TX packets"},
            {ElementType::String, "CAN Bus2"}, {ElementType::Int32, "RX2 packets"}, {ElementType::Int32, "TX2 packets"},
            {ElementType::String, "Status"}
        }},
        {"POWER", {
            {ElementType::Float32, "Voltage"}, {ElementType::Float32, "Temp"},
            {ElementType::Float32, "Current 0"}, {ElementType::Float32, "Current 1"},
            {ElementType::Float32, "Current 2"}, {ElementType::Float32, "Current 3"},
            {ElementType::Float32, "Current 4"}, {ElementType::Float32, "Current 5"}, {ElementType::Float32, "Current 6"}
        }},
        {"POWER2", {
            {ElementType::Float32, "Current 7"}, {ElementType::Float32, "Current 8"}, {ElementType::Float32, "Current 9"},
            {ElementType::Float32, "Current 10"}, {ElementType::Float32, "Current 11"}, {ElementType::Float32, "Current 12"},
            {ElementType::Float32, "Current 13"}, {ElementType::Float32, "Current 14"}, {ElementType::Float32, "Current 15"}
        }},
        {"DRIVETRAIN", {
            {ElementType::Float32, "F1 Vel"}, {ElementType::Float32, "F1 RPM"}, {ElementType::Float32, "F1 Speed"},
            {ElementType::Float32, "F2 Vel"}, {ElementType::Float32, "F2 RPM"}, {ElementType::Float32, "F2 Speed"},
            {ElementType::Float32, "F3 Vel"}, {ElementType::Float32, "F3 RPM"}, {ElementType::Float32, "F3 Speed"},
            {ElementType::Float32, "F4 Vel"}, {ElementType::Float32, "F4 RPM"}, {ElementType::Float32, "F4 Speed"}
        }}
    };
    return element_definitions;
}

// --- Initialization ---
void initialize_maps() {
    initialize_bool_map(get_talon_values(), get_talon_keys());
    initialize_bool_map(get_falcon_values(), get_falcon_keys());
    initialize_bool_map(get_linear_values(), get_linear_keys());
    initialize_bool_map(get_power_values(), get_power_keys());
    initialize_bool_map(get_power2_values(), get_power2_keys());
    initialize_bool_map(get_autonomy_values(), get_autonomy_keys());
    initialize_bool_map(get_zed_values(), get_zed_keys());
    initialize_bool_map(get_communication_values(), get_communication_keys());
    initialize_bool_map(get_drivetrain_values(), get_drivetrain_keys());
}

// --- Helper Functions ---
std::vector<std::string> getKeys(const std::string& label) {
    if (label == "Power2") return get_power2_keys();
    for (const auto& [prefix, keys_ptr] : get_key_vectors())
        if (label.rfind(prefix, 0) == 0)
            return *keys_ptr;
    return get_talon_keys();
}

std::map<std::string, bool>& getMap(std::string label) {
    if (label.rfind("Talon", 0) == 0) return get_talon_values();
    if (label.rfind("Falcon", 0) == 0) return get_falcon_values();
    if (label.rfind("Linear", 0) == 0) return get_linear_values();
    if (label.rfind("Autonomy", 0) == 0) return get_autonomy_values();
    if (label.rfind("Communication", 0) == 0) return get_communication_values();
    if (label.rfind("Power2", 0) == 0) return get_power2_values();
    if (label.rfind("Power", 0) == 0) return get_power_values();
    if (label.rfind("Zed", 0) == 0) return get_zed_values();
    if (label.rfind("Drivetrain", 0) == 0) return get_drivetrain_values();
    return get_talon_values();
}

std::string getNameFromPrefix(std::string label) {
    if (label.rfind("TALON", 0) == 0) return "Talon";
    if (label.rfind("FALCON", 0) == 0) return "Falcon";
    if (label.rfind("LINEAR", 0) == 0) return "Linear";
    if (label.rfind("AUTONOMY", 0) == 0) return "Autonomy";
    if (label.rfind("COMMUNICATION", 0) == 0) return "Communication";
    if (label.rfind("POWER2", 0) == 0) return "Power2";
    if (label.rfind("POWER", 0) == 0) return "Power";
    if (label.rfind("ZED", 0) == 0) return "Zed";
    if (label.rfind("TEST", 0) == 0) return "Test";
    if (label.rfind("DRIVETRAIN", 0) == 0) return "Drivetrain";
    return "Talon";
}

Gdk::RGBA parse_color(const std::string& color_str) {
    Gdk::RGBA color;
    color.set(color_str);
    return color;
}

std::string to_color_string(const Gdk::RGBA& color) {
    return color.to_string();
}

void addElementToInfoFrame(InfoFrame* frame, const Element& element) {
    frame->addItem(element.label);
    const auto& data = element.data.front();
    switch (element.type) {
        case TYPE::BOOLEAN:   frame->setItem(element.label, data.boolean); break;
        case TYPE::INT8:      frame->setItem(element.label, data.int8); break;
        case TYPE::UINT8:     frame->setItem(element.label, data.uint8); break;
        case TYPE::INT16:     frame->setItem(element.label, data.int16); break;
        case TYPE::UINT16:
            if (element.label == "Bus Voltage" || element.label == "Output Current")
                frame->setItem(element.label, data.uint16 / 100.0f);
            else
                frame->setItem(element.label, data.uint16);
            break;
        case TYPE::INT32:     frame->setItem(element.label, data.int32); break;
        case TYPE::UINT32:    frame->setItem(element.label, data.uint32); break;
        case TYPE::INT64:     frame->setItem(element.label, data.int64); break;
        case TYPE::UINT64:    frame->setItem(element.label, data.uint64); break;
        case TYPE::FLOAT32:   frame->setItem(element.label, data.float32); break;
        case TYPE::FLOAT64:   frame->setItem(element.label, data.float64); break;
        case TYPE::STRING: {
            std::string text;
            for (const auto& c : element.data) text += c.character;
            frame->setItem(element.label, text);
            break;
        }
        default: break;
    }
}