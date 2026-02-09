#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <linux/if_packet.h>
#include <thread>
#include <chrono>
#include <vector>
#include <linux/reboot.h>
#include <sys/reboot.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <cstdint>
#include <zlib.h>

#include <messages/msg/power.hpp>
#include <messages/msg/key_state.hpp>
#include <messages/msg/hat_state.hpp>  
#include <messages/msg/button_state.hpp>
#include <messages/msg/axis_state.hpp>
#include <messages/msg/talon_status.hpp>
#include <messages/msg/zed_position.hpp>
#include <messages/msg/linear_status.hpp>
#include <messages/msg/autonomy_status.hpp>
#include <messages/msg/falcon_status.hpp>
#include <messages/msg/system_status.hpp>
#include <messages/msg/drivetrain_status.hpp>

#include <BinaryMessage.hpp>
#include <Heartbeat.hpp>
#include <RobotState.hpp>
#include <NetworkUtils.hpp>
#include <MessageUtils.hpp>
#include "AegisController.hpp"
#include "utils/utils.hpp"
#include "EthernetHBThread.hpp"

#include <iostream>
#include <cstring>
#include <net/if.h>
#include <netdb.h>

#define ETHERNET_IFACE "enP8p1s0"
#define PORT 31337

const std::string TARGET_IP = "192.168.50.11";
std::string robotName="unnamed";
std::string interfaceName = "wlP1p1s0";
bool broadcast=true;

std::atomic<uint32_t> global_seq{0};
std::atomic<uint64_t> last_ros_update_time {0};
std::atomic<uint64_t> last_client_tx_time_ms {0};

/** @file
 * @brief Node for handling communication between the client and the rover.
 *  
 * This node receives information published by the power_distribution_panel node, motor nodes, and the logic node
 * wraps the information into topics, then publishes the topics.  
 * Currently this node is missing the callback functions for the motors. 
 * The topics that the node subscribes to are as follows:
 * \li \b power
 * \li \b talon_10_info
 * \li \b talon_11_info
 * \li \b talon_12_info
 * \li \b talon_13_info
 * \li \b talon_14_info
 * \li \b talon_15_info
 * \li \b talon_16_info
 * \li \b talon_17_info
 * \li \b zed_position
 * \li \b autonomy_status
 * 
 * The topics that are being published are as follows:
 * \li \b joystick_axis
 * \li \b joystick_button
 * \li \b joystick_hat
 * \li \b key
 * \li \b STOP
 * \li \b GO
 * 
 * To read more about the nodes that subscribe to this one
 * \see logic_node.cpp
 * 
 * 
 * */

struct sockaddr_in address; 
socklen_t addrlen = sizeof(address); 
std_msgs::msg::Empty empty;
bool silentRunning=true;
int new_socket;
rclcpp::Node::SharedPtr nodeHandle;
std_msgs::msg::Empty heartbeat;
int total = 0;

int rssi = 0;
bool usingCAN1 = false;
bool debug = false;
bool init = false;

#define LOWER_THRESH 67
#define UPPER_THRESH 80
#define CRIT_THRESH 90


Falcon falcon1, falcon2, falcon3, falcon4;
Talon talon1, talon2, talon3, talon4;
Linear linear1, linear2, linear3, linear4;
AutonomyState autonomyState;
ZedState zedState;
DrivetrainState drivetrainState;
SystemState systemState;

std::unique_ptr<HeartbeatLink> orinLink;
std::unique_ptr<CanLink> orinCanLink;
std::shared_ptr<AegisController> orinController;
std::mutex orinMutex;
RemoteStatus orinRemoteStatus;
bool orinRawData = false;
SystemStatus orinSysStatus = BOOT; 
HandshakeStatus orinHandshakeStatus = IDLE_HANDSHAKE;
ErrorCode orinErrorCode = NO_ERROR;

std::unique_ptr<EthernetHBThread> orinEthHB;

CanHeartbeatPayload nano_hb {0x02, 0, 0, 0};

#define ORIN_PORT 31339
#define NANO_PORT 31340
#define LOCAL_IP "127.0.0.1"
#define REMOTE_IP "192.168.50.11"
std::atomic<bool> is_primary {true};

float voltage = 0.0f;
float temperature = 0.0f;
std::array<float, 16> currents{};

uint64_t get_time_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

/** * @brief Resets all internal state trackers to impossible values.
 * * This forces the 'update_if_changed' logic to detect a difference 
 * the next time a ROS2 callback fires, causing a full transmission 
 * of all data to the newly connected client.
 */
void forceDataResync() {
    // Helper lambda to reset a Falcon struct
    auto resetFalcon = [](Falcon& f) {
        f.device_id = 255;
        f.voltage = 0xFFFF;
        f.current = 0xFFFF;
        f.output_percent = -999.0f; 
        f.temperature = 255;
        f.sensor_position = -999999.0f;
        f.sensor_velocity = -999999.0f;
        f.max_current = -1.0f;
        f.temp_disable = !f.temp_disable;
        f.error = !f.error;
    };

    // Helper lambda to reset a Talon struct
    auto resetTalon = [](Talon& t) {
        t.device_id = 255;
        t.voltage = 0xFFFF;
        t.current = 0xFFFF;
        t.output_percent = -999.0f;
        t.temperature = 255;
        t.sensor_position = -999999.0f;
        t.sensor_velocity = -999999.0f;
        t.max_current = -1.0f;
        t.temp_disable = !t.temp_disable;
    };

    // Helper lambda to reset a Linear struct
    auto resetLinear = [](Linear& l) {
        l.motor_number = 255;
        l.speed = -999.0f;
        l.potentiometer = 0xFFFF;
        l.time_without_change = 255;
        l.max = 0xFFFF;
        l.min = 0xFFFF;
        l.error = "FORCE_RESYNC";
        l.distance = -999.0f;
    };

    resetFalcon(falcon1); resetFalcon(falcon2); resetFalcon(falcon3); resetFalcon(falcon4);
    resetTalon(talon1); resetTalon(talon2); resetTalon(talon3); resetTalon(talon4);
    resetLinear(linear1); resetLinear(linear2); resetLinear(linear3); resetLinear(linear4);

    // Reset Autonomy State
    autonomyState.robot_state = "RESYNC";
    autonomyState.excavation_state = "RESYNC";
    autonomyState.error_state = "RESYNC";
    autonomyState.diagnostics_state = "RESYNC";
    autonomyState.tilt_state = "RESYNC";
    autonomyState.dump_state = "RESYNC";
    autonomyState.bucket_state = "RESYNC";
    autonomyState.arms_state = "RESYNC";
    autonomyState.dest_x = -99999.0f;
    autonomyState.dest_z = -99999.0f;

    // Reset Zed State
    zedState.x = -99999.0f;
    zedState.y = -99999.0f;
    zedState.z = -99999.0f;
    zedState.roll = -999.0f;
    zedState.pitch = -999.0f;
    zedState.yaw = -999.0f;
    zedState.aruco = !zedState.aruco;

    // Reset Drivetrain State
    drivetrainState.f1_vel = -99999.0f; drivetrainState.f1_rpm = -99999.0f; drivetrainState.f1_speed = -99999.0f;
    drivetrainState.f2_vel = -99999.0f; drivetrainState.f2_rpm = -99999.0f; drivetrainState.f2_speed = -99999.0f;
    drivetrainState.f3_vel = -99999.0f; drivetrainState.f3_rpm = -99999.0f; drivetrainState.f3_speed = -99999.0f;
    drivetrainState.f4_vel = -99999.0f; drivetrainState.f4_rpm = -99999.0f; drivetrainState.f4_speed = -99999.0f;

    // Reset System State
    systemState.rssi = -1;
    systemState.wifi = "RESYNC";
    systemState.can_bus = "RESYNC";
    systemState.rx_packets = -1;
    
    // Reset Power Arrays (Global vars)
    voltage = -1.0f;
    temperature = -1.0f;
    currents.fill(-1.0f);
}

void updatePrimaryState(bool state) {
    if (is_primary != state) {
        if (state) {
            RCLCPP_INFO(nodeHandle->get_logger(), "Role Switched: PRIMARY (Publishing enabled)");
        }
        else {
            RCLCPP_INFO(nodeHandle->get_logger(), "Role Switched: SECONDARY (Publishing disabled)");
        }
        is_primary = state;
    }
}

void primaryStateCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    updatePrimaryState(msg->data);
}

/**
 * @brief Serializes, checksums, and conditionally compresses a BinaryMessage before sending.
 * * This function first compresses the data. If the compressed size is smaller than
 * the original, it sends a compressed payload prefixed with a '1' flag and the
 * original data size. Otherwise, it sends the original uncompressed data prefixed
 * with a '0' flag.
 *
 * Compressed Payload:   [ 1-byte flag = 1 ] [ 4-byte original size ] [ N-bytes compressed data ]
 * Uncompressed Payload: [ 1-byte flag = 0 ] [ N-bytes original data ]
 * * @param message The BinaryMessage object to be sent.
 */
void send(BinaryMessage message) {
    if (!is_primary.load()) return;

    // 1. Get the raw bytes and apply the checksum.
    std::shared_ptr<std::list<uint8_t>> byteList = message.getBytes();
    checksum_encode(byteList);

    std::vector<uint8_t> uncompressed_bytes(byteList->begin(), byteList->end());
    uLong uncompressed_size = uncompressed_bytes.size();

    if (uncompressed_size == 0) return;

    // 2. Attempt compression.
    uLong compressed_buffer_size = compressBound(uncompressed_size);
    std::vector<uint8_t> compressed_bytes(compressed_buffer_size);

    int compression_result = compress2(
        compressed_bytes.data(), &compressed_buffer_size,
        uncompressed_bytes.data(), uncompressed_size, Z_DEFAULT_COMPRESSION);

    std::vector<uint8_t> payload;

    // 3. Check if compression was successful AND beneficial.
    if (compression_result == Z_OK && compressed_buffer_size < uncompressed_size) {
        // ---- COMPRESSION IS BENEFICIAL ----
        // Flag (1), Original Size (4 bytes), Compressed Data (N bytes)
        payload.reserve(1 + 4 + compressed_buffer_size);
        
        // Add the '1' flag to indicate compression
        payload.push_back(1); 

        // Add the 4-byte uncompressed size header (network byte order)
        payload.push_back((uncompressed_size >> 24) & 0xFF);
        payload.push_back((uncompressed_size >> 16) & 0xFF);
        payload.push_back((uncompressed_size >> 8) & 0xFF);
        payload.push_back(uncompressed_size & 0xFF);
        
        // Add the compressed data
        payload.insert(payload.end(), compressed_bytes.begin(), compressed_bytes.begin() + compressed_buffer_size);

    } else {
        // Flag (1), Original Data (N bytes)
        payload.reserve(1 + uncompressed_size);

        // Add the '0' flag to indicate raw, uncompressed data
        payload.push_back(0); 

        // Add the original data
        payload.insert(payload.end(), uncompressed_bytes.begin(), uncompressed_bytes.end());
    }

    // 4. Send the chosen payload.
    try {
        if (payload.empty()) return;
        sendto(new_socket, payload.data(), payload.size(), 0, (struct sockaddr *)&address, addrlen);
        last_client_tx_time_ms.store(get_time_ms(), std::memory_order_relaxed);
    } catch (...) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "ERROR: Exception when trying to send data to client");
    }
}

static void sendClientHeartbeat() {
    if (new_socket <= 0) return;
    if (broadcast) return;
    const uint8_t hb[1] = {0};
    sendto(new_socket, hb, sizeof(hb), 0, (struct sockaddr *)&address, addrlen);
    last_client_tx_time_ms.store(get_time_ms(), std::memory_order_relaxed);
}

void send(std::string messageLabel, const messages::msg::FalconStatus::SharedPtr falconStatus, Falcon& falcon) {
    if (silentRunning) return;

    bool message_changed = false;
    BinaryMessage message(messageLabel);

    uint16_t new_voltage = falconStatus->bus_voltage * 100.0;
    uint16_t new_current = falconStatus->output_current * 100.0;
    uint8_t new_temperature = (uint8_t)falconStatus->temperature;
    uint8_t new_device_id = (uint8_t)falconStatus->device_id;

    update_if_changed(message, message_changed, falcon.device_id,      new_device_id,                 "Device ID");
    update_if_changed(message, message_changed, falcon.voltage,         new_voltage,                   "Bus Voltage");
    update_if_changed(message, message_changed, falcon.current,         new_current,                   "Output Current");
    update_if_changed(message, message_changed, falcon.output_percent,  falconStatus->output_percent,  "Output Percent");
    update_if_changed(message, message_changed, falcon.temperature,     new_temperature,               "Temperature");
    update_if_changed(message, message_changed, falcon.sensor_position, falconStatus->sensor_position, "Sensor Position");
    update_if_changed(message, message_changed, falcon.sensor_velocity, falconStatus->sensor_velocity, "Sensor Velocity");
    update_if_changed(message, message_changed, falcon.max_current,     falconStatus->max_current,     "Max Current");
    update_if_changed(message, message_changed, falcon.temp_disable,    falconStatus->temp_disable,    "Temp Disable");
    update_if_changed(message, message_changed, falcon.error,           falconStatus->error,           "Error");

    if (message_changed) {
        send(message);
    }
}


void send(std::string messageLabel, const messages::msg::TalonStatus::SharedPtr talonStatus, Talon& talon) {
    if (silentRunning) return;

    bool message_changed = false;
    BinaryMessage message(messageLabel);

    uint16_t new_voltage = talonStatus->bus_voltage * 100.0;
    uint16_t new_current = talonStatus->output_current * 100.0;
    float new_sensor_pos = talonStatus->sensor_position; 

    update_if_changed(message, message_changed, talon.voltage,        new_voltage,                  "Bus Voltage");
    update_if_changed(message, message_changed, talon.current,        new_current,                  "Output Current");
    update_if_changed(message, message_changed, talon.output_percent, talonStatus->output_percent,  "Output Percent");
    update_if_changed(message, message_changed, talon.temperature,    (uint8_t)talonStatus->temperature, "Temperature");
    update_if_changed(message, message_changed, talon.sensor_position,new_sensor_pos,               "Sensor Position");
    update_if_changed(message, message_changed, talon.sensor_velocity,talonStatus->sensor_velocity, "Sensor Velocity");
    update_if_changed(message, message_changed, talon.max_current,    talonStatus->max_current,     "Max Current");
    update_if_changed(message, message_changed, talon.temp_disable,   talonStatus->temp_disable,    "Temp Disable");

    if (message_changed) {
        send(message);
    }
}

void send(std::string messageLabel, const messages::msg::Power::SharedPtr power) {
    if (silentRunning) return;

    const std::array<float, 16> power_currents = {
        power->current0,  power->current1,  power->current2,  power->current3,
        power->current4,  power->current5,  power->current6,  power->current7,
        power->current8,  power->current9,  power->current10, power->current11,
        power->current12, power->current13, power->current14, power->current15
    };

    bool message1_changed = false;
    BinaryMessage message1(messageLabel);

    update_if_changed(message1, message1_changed, voltage, power->voltage, "Voltage");
    update_if_changed(message1, message1_changed, temperature, power->temperature, "Temp");

    for (int i = 0; i <= 6; ++i) {
        update_if_changed(message1, message1_changed, currents[i], power_currents[i], "Current " + std::to_string(i));
    }

    if (message1_changed) {
        send(message1);
    }

    bool message2_changed = false;
    BinaryMessage message2("Power2");

    for (int i = 7; i <= 15; ++i) {
        update_if_changed(message2, message2_changed, currents[i], power_currents[i], "Current " + std::to_string(i));
    }

    if (message2_changed) {
        send(message2);
    }
}


void send(std::string messageLabel, const messages::msg::LinearStatus::SharedPtr linearStatus, Linear& linear) {
    if (silentRunning) return;

    bool message_changed = false;
    BinaryMessage message(messageLabel);

    uint8_t new_motor_number = (uint8_t)linearStatus->motor_number;
    uint16_t new_potentiometer = (uint16_t)linearStatus->potentiometer;
    uint8_t new_time_without_change = (uint8_t)linearStatus->time_without_change;
    uint16_t new_max = (uint16_t)linearStatus->max;
    uint16_t new_min = (uint16_t)linearStatus->min;

    update_if_changed(message, message_changed, linear.motor_number,       new_motor_number,            "Motor Number");
    update_if_changed(message, message_changed, linear.speed,              linearStatus->speed,         "Speed");
    update_if_changed(message, message_changed, linear.potentiometer,      new_potentiometer,           "Potentiometer");
    update_if_changed(message, message_changed, linear.time_without_change,new_time_without_change,     "Time Without Change");
    update_if_changed(message, message_changed, linear.max,                new_max,                     "Max");
    update_if_changed(message, message_changed, linear.min,                new_min,                     "Min");
    update_if_changed(message, message_changed, linear.error,              linearStatus->error,         "Error");
    update_if_changed(message, message_changed, linear.at_min,             linearStatus->at_min,        "At Min");
    update_if_changed(message, message_changed, linear.at_max,             linearStatus->at_max,        "At Max");
    update_if_changed(message, message_changed, linear.distance,           linearStatus->distance,      "Distance");
    update_if_changed(message, message_changed, linear.sensorless,         linearStatus->sensorless,    "Sensorless");

    if (message_changed) {
        send(message);
    }
}


void send(std::string messageLabel, const messages::msg::AutonomyStatus::SharedPtr autonomy) {
    if (silentRunning) return;

    bool message_changed = false;
    BinaryMessage message(messageLabel);

    update_if_changed(message, message_changed, autonomyState.robot_state,      autonomy->robot_state,      "Robot State");
    update_if_changed(message, message_changed, autonomyState.excavation_state, autonomy->excavation_state, "Excavation State");
    update_if_changed(message, message_changed, autonomyState.error_state,      autonomy->error_state,      "Error State");
    update_if_changed(message, message_changed, autonomyState.diagnostics_state,autonomy->diagnostics_state,"Diagnostics State");
    update_if_changed(message, message_changed, autonomyState.tilt_state,       autonomy->tilt_state,       "Tilt State");
    update_if_changed(message, message_changed, autonomyState.dump_state,       autonomy->dump_state,       "Dump State");
    update_if_changed(message, message_changed, autonomyState.bucket_state,     autonomy->bucket_state,     "Level Bucket");
    update_if_changed(message, message_changed, autonomyState.arms_state,       autonomy->arms_state,       "Level Arms");
    update_if_changed(message, message_changed, autonomyState.dest_x,           autonomy->dest_x,           "Dest X");
    update_if_changed(message, message_changed, autonomyState.dest_z,           autonomy->dest_z,           "Dest Z");

    if (message_changed) {
        RCLCPP_INFO(nodeHandle->get_logger(), "Sending message");
        send(message);
    }
}


// 30 Hz
int zedCounter = 0;
/** @brief Callback function that publishes position data to the client
 * 
 * This function is called when the node receives position data from the
 * autonomy node.  This data is then published to the client to be displayed
 * on the GUI. 
 * @param zedPosition 
 */
void zedPositionCallback(const messages::msg::ZedPosition::SharedPtr zedPosition){
    orinController->receivedZedTracking();
    if(silentRunning)return;
    if(rssi > UPPER_THRESH)
        return;
    zedCounter++;
    if(zedCounter % 15 != 0)
        return;

    bool message_changed = false;
    BinaryMessage message("Zed");

    update_if_changed(message, message_changed, zedState.x,     zedPosition->x,             "X");
    update_if_changed(message, message_changed, zedState.y,     zedPosition->y,             "Y");
    update_if_changed(message, message_changed, zedState.z,     zedPosition->z,             "Z");
    update_if_changed(message, message_changed, zedState.roll,  zedPosition->roll,          "roll");
    update_if_changed(message, message_changed, zedState.pitch, zedPosition->pitch,         "pitch");
    update_if_changed(message, message_changed, zedState.yaw,   zedPosition->yaw,           "yaw");
    update_if_changed(message, message_changed, zedState.aruco, zedPosition->aruco_visible, "aruco");

    if(message_changed){
        send(message);
    }
}

// 10 Hz
int systemCounter = 0;
void systemStatusCallback(const messages::msg::SystemStatus::SharedPtr status) {
    orinController->receivedStatusMonitor();
    if (silentRunning) return;

    systemCounter++;
    if (systemCounter % 5 != 0) return;

    bool message_changed = false;
    BinaryMessage message("Communication");

    std::string new_wifi_status;
    if (status->rssi < LOWER_THRESH) {
        new_wifi_status = "NORMAL";
    }
    else if (status->rssi < UPPER_THRESH) {
        new_wifi_status = "DEGRADED";
    }
    else if (status->rssi < CRIT_THRESH) {
        new_wifi_status = "INTERFERENCE";
    }
    else {
        new_wifi_status = "NON-FUNCTIONAL";
    }

    update_if_changed(message, message_changed, systemState.rssi,         status->rssi,          "RSSI");
    update_if_changed(message, message_changed, systemState.wifi,         new_wifi_status,       "Wi-Fi");
    update_if_changed(message, message_changed, systemState.can_bus,      status->can_message,   "CAN Bus");
    update_if_changed(message, message_changed, systemState.using_can1,   status->using_can1,    "Using CAN1");
    update_if_changed(message, message_changed, systemState.rx_packets,   status->rx_packets,    "RX packets");
    update_if_changed(message, message_changed, systemState.tx_packets,   status->tx_packets,    "TX packets");
    update_if_changed(message, message_changed, systemState.can_bus2,     status->can2_message,  "CAN Bus2");
    update_if_changed(message, message_changed, systemState.rx_packets2,  status->rx2_packets,   "RX2 packets");
    update_if_changed(message, message_changed, systemState.tx_packets2,  status->tx2_packets,   "TX2 packets");
    update_if_changed(message, message_changed, systemState.first_motor,  status->first_motor,   "First Motor");
    update_if_changed(message, message_changed, systemState.second_motor, status->second_motor,  "Second Motor");
    update_if_changed(message, message_changed, systemState.num_breaks,   status->num_breaks,    "Num Breaks");

    if (message_changed) {
        send(message);
    }
}


// 30 Hz
int drivetrainCounter = 0;
void drivetrainStatusCallback(const messages::msg::DrivetrainStatus::SharedPtr status){
    if(silentRunning)return;
    drivetrainCounter++;
    if(drivetrainCounter % 10 != 0 )return;
    bool message_changed = false;
    BinaryMessage message("Drivetrain");

    update_if_changed(message, message_changed, drivetrainState.f1_vel,   status->falcon1_velocity,      "F1 Vel");
    update_if_changed(message, message_changed, drivetrainState.f1_rpm,   status->falcon1_rpm,           "F1 RPM");
    update_if_changed(message, message_changed, drivetrainState.f1_speed, status->falcon1_ground_speed,  "F1 Speed");
    update_if_changed(message, message_changed, drivetrainState.f2_vel,   status->falcon2_velocity,      "F2 Vel");
    update_if_changed(message, message_changed, drivetrainState.f2_rpm,   status->falcon2_rpm,           "F2 RPM");
    update_if_changed(message, message_changed, drivetrainState.f2_speed, status->falcon2_ground_speed,  "F2 Speed");
    update_if_changed(message, message_changed, drivetrainState.f3_vel,   status->falcon3_velocity,      "F3 Vel");
    update_if_changed(message, message_changed, drivetrainState.f3_rpm,   status->falcon3_rpm,           "F3 RPM");
    update_if_changed(message, message_changed, drivetrainState.f3_speed, status->falcon3_ground_speed,  "F3 Speed");
    update_if_changed(message, message_changed, drivetrainState.f4_vel,   status->falcon4_velocity,      "F4 Vel");
    update_if_changed(message, message_changed, drivetrainState.f4_rpm,   status->falcon4_rpm,           "F4 RPM");
    update_if_changed(message, message_changed, drivetrainState.f4_speed, status->falcon4_ground_speed,  "F4 Speed");
    
    if (message_changed) {
        send(message);
    }
}


// 10 Hz
int powerCounter = 0;
/** @brief Callback function for the power topic.
 * 
 * This function is called when the node receives a
 * topic with the name power. This function
 * extracts the information given from the power topic
 * and places data into a payload to be sent.
 *  
 * \see .power_distribution_panel.cpp
 * @param power
 * @return void
 * */
void powerCallback(const messages::msg::Power::SharedPtr power){
    //RCLCPP_INFO(nodeHandle->get_logger(), "power callback");
    powerCounter++;
    if(powerCounter % 5 == 0)
        if(rssi < UPPER_THRESH)
            send("Power",power);
}


// 100 Hz
/** @brief Callback function for the Talon topic
 * 
 * This function receives the talonStatus message published by the first 
 * Talon and uses the send function to send the data to the client side
 * GUI.
 * @param talonStatus
 * @return void
 * */
void talonStatusCallback(const std::string& name, const messages::msg::TalonStatus::SharedPtr talonStatus, int& counter, Talon& talon){
    //RCLCPP_INFO(nodeHandle->get_logger(), "talon1 callback");
    counter++;
    if(counter % 20 == 0)
        if(rssi < CRIT_THRESH)
            send(name, talonStatus, talon);
}


void sendFalconCrit(std::string messageLabel, const messages::msg::FalconStatus::SharedPtr talonStatus, Falcon& falcon){
    if(silentRunning)return;
    //RCLCPP_INFO(nodeHandle->get_logger(), "send talon");
    if(talonStatus->output_percent == falcon.output_percent)
        return;

    BinaryMessage message(messageLabel);
    message.addElementFloat32("Output Percent",talonStatus->output_percent);
    send(message);
}


// 20 Hz
/** @brief Callback function for the Talon topic
 * 
 * This function receives the talonStatus message published by the first 
 * Talon and uses the send function to send the data to the client side
 * GUI.
 * @param talonStatus
 * @return void
 * */
void falconStatusCallback(const std::string& name, const messages::msg::FalconStatus::SharedPtr talonStatus, int& counter, Falcon& falcon){
    //RCLCPP_INFO(nodeHandle->get_logger(), "falcon1 callback");
    counter++;
    if(counter % 20 == 0){
        if(rssi < CRIT_THRESH)
            send(name,talonStatus, falcon);
    }
    else{
        if(rssi < CRIT_THRESH)
            sendFalconCrit(name, talonStatus, falcon);
    }
}


// 60 Hz
/** @brief Callback function for the LinearStatus topic
 * 
 * This function receives the linearStatus message published by the excavation
 * node and uses the send data to send the data to the client side GUI.
 * @param name
 * @param linearStatus 
 */
void linearStatusCallback(const std::string& name, const messages::msg::LinearStatus::SharedPtr linearStatus, int& counter, Linear& linear){
    //RCLCPP_INFO(nodeHandle->get_logger(), "%s callback", name.c_str());
    counter++;
    if(counter % 30 == 0)
        if(rssi < UPPER_THRESH)
            send(name, linearStatus, linear);
}


// 30 Hz
int autonomyCounter = 0;
void autonomyStatusCallback(const messages::msg::AutonomyStatus::SharedPtr autonomyStatus){
    orinController->receivedAutonomy();
    //RCLCPP_INFO(nodeHandle->get_logger(), "autonomy callback");
    autonomyCounter++;
    if(autonomyCounter % 15 == 0)
        if(rssi < CRIT_THRESH)
            send("Autonomy", autonomyStatus);
}


/** @brief Creates socketDescriptor for socket connection.
 * 
 * This function is called when the node
 * tries to setup the socket connection between the rover and client.
 * This function creates the socketDescriptor for the socket connection.
 * Uses the getAddressString function.
 * */
void broadcastIP(){
    while(true){
        if(broadcast){
            std::string addressString=getAddressString(AF_INET,interfaceName);

            std::string message(robotName+"@"+addressString);
            //std::cout << message << std::endl << std::flush;

            int socketDescriptor=socket(AF_INET, SOCK_DGRAM, 0);
            if(socketDescriptor>=0){
                struct sockaddr_in socketAddress;
                socketAddress.sin_family=AF_INET;
                socketAddress.sin_addr.s_addr = inet_addr("226.1.1.1");
                socketAddress.sin_port = htons(4321);

                struct in_addr localInterface;
                localInterface.s_addr = inet_addr(addressString.c_str());
                if(setsockopt(socketDescriptor, IPPROTO_IP, IP_MULTICAST_IF, (char*)&localInterface, sizeof(localInterface))>=0){
                    sendto(socketDescriptor,message.c_str(),message.length(),0,(struct sockaddr*)&socketAddress, (socklen_t)sizeof(socketAddress));
                }
            }
            close(socketDescriptor);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

//HeartbeatLink hb_link(31339, "10.42.0.2", 31339);
std::thread comms_thread;
std::atomic<bool> node_running {true};

std::mutex comms_mutex;

void network_worker() {
    while (node_running) {
        // 1. Read Incoming Packets (Drain the buffer)
        // We loop until no more packets are waiting to prevent buffer overflow
        while (orinLink->spin_once());
        orinController->checkTimers();
        orinCanLink->read_heartbeat(nano_hb);

        uint64_t now = get_time_ms();
        if (now - last_ros_update_time > 100) {
            // The comms thread has crashed and we need to no longer send a heartbeat
            continue; 
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);

    nodeHandle = rclcpp::Node::make_shared("communication");
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting communication node");

    robotName = utils::getParameter<std::string>(nodeHandle, "robot_name", "not named");
    debug = utils::getParameter<bool>(nodeHandle, "debug", false);
    bool useLocal = utils::getParameter<bool>(nodeHandle, "local", false);

    orinRemoteStatus.UP = false;
    orinRemoteStatus.WIFI_UP = false;
    orinRemoteStatus.CAN0_UP = false;
    orinRemoteStatus.CAN1_UP = false;

    if(useLocal)
        orinLink = std::make_unique<HeartbeatLink>(ORIN_PORT, LOCAL_IP, NANO_PORT);
    else
        orinLink = std::make_unique<HeartbeatLink>(ORIN_PORT, REMOTE_IP, NANO_PORT);
    
    // 1. Initialize Heartbeat Link
    if (!orinLink->init()) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to init Heartbeat Link!");
        return -1;
    }
    orinCanLink = std::make_unique<CanLink>();
    orinController = std::make_shared<AegisController>(
        nodeHandle, *orinLink, *orinCanLink, orinMutex, orinRemoteStatus, orinRawData, orinSysStatus, orinHandshakeStatus, orinErrorCode, updatePrimaryState
    );
    using namespace std::placeholders;
    orinLink->set_data_callback(
        std::bind(&AegisController::on_packet_received, orinController, _1, _2, _3)
    );
    orinEthHB = std::make_unique<EthernetHBThread>(*orinLink, std::chrono::milliseconds(10));
    orinEthHB->start();
    orinController->initAegis();
    comms_thread = std::thread(network_worker);
    RCLCPP_INFO(nodeHandle->get_logger(), "Comms Thread Started.");

    auto joystickAxisPublisher = nodeHandle->create_publisher<messages::msg::AxisState>("joystick_axis", 1);
    auto joystickHatPublisher = nodeHandle->create_publisher<messages::msg::HatState>("joystick_hat",1);
    auto joystickButtonPublisher = nodeHandle->create_publisher<messages::msg::ButtonState>("joystick_button",1);
    auto keyPublisher = nodeHandle->create_publisher<messages::msg::KeyState>("key",1);
    auto stopPublisher = nodeHandle->create_publisher<std_msgs::msg::Empty>("STOP",1);
    auto goPublisher=nodeHandle->create_publisher<std_msgs::msg::Empty>("GO",1);
    auto commHeartbeatPublisher = nodeHandle->create_publisher<std_msgs::msg::Empty>("comm_heartbeat",1);

    auto powerSubscriber = nodeHandle->create_subscription<messages::msg::Power>("power",1,powerCallback);
    int talon1Counter = 0, talon2Counter = 0, talon3Counter = 0, talon4Counter = 0;
    auto talon1Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>(
            "talon_14_info", 1,
            [&](const messages::msg::TalonStatus::SharedPtr msg) {
                talonStatusCallback("Talon 1", msg, talon1Counter, talon1);
            });

    auto talon2Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>(
            "talon_15_info", 1,
            [&](const messages::msg::TalonStatus::SharedPtr msg) {
                talonStatusCallback("Talon 2", msg, talon2Counter, talon2);
            });

    auto talon3Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>(
            "talon_16_info", 1,
            [&](const messages::msg::TalonStatus::SharedPtr msg) {
                talonStatusCallback("Talon 3", msg, talon3Counter, talon3);
            });

    auto talon4Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>(
            "talon_17_info", 1,
            [&](const messages::msg::TalonStatus::SharedPtr msg) {
                talonStatusCallback("Talon 4", msg, talon4Counter, talon4);
            });

    int falcon1Counter = 0, falcon2Counter = 0, falcon3Counter = 0, falcon4Counter = 0;
    auto falcon1Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>(
            "talon_10_info", 1,
            [&](const messages::msg::FalconStatus::SharedPtr msg) {
                falconStatusCallback("Falcon 1", msg, falcon1Counter, falcon1);
            });

    auto falcon2Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>(
            "talon_11_info", 1,
            [&](const messages::msg::FalconStatus::SharedPtr msg) {
                falconStatusCallback("Falcon 2", msg, falcon2Counter, falcon2);
            });

    auto falcon3Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>(
            "talon_12_info", 1,
            [&](const messages::msg::FalconStatus::SharedPtr msg) {
                falconStatusCallback("Falcon 3", msg, falcon3Counter, falcon3);
            });

    auto falcon4Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>(
            "talon_13_info", 1,
            [&](const messages::msg::FalconStatus::SharedPtr msg) {
                falconStatusCallback("Falcon 4", msg, falcon4Counter, falcon4);
            });

    int linear1Counter = 0, linear2Counter = 0, linear3Counter = 0, linear4Counter = 0;
    auto linearStatus1Subscriber = nodeHandle->create_subscription<messages::msg::LinearStatus>(
            "linearStatus1", 1,
            [&](const messages::msg::LinearStatus::SharedPtr msg) {
                linearStatusCallback("Linear 1", msg, linear1Counter, linear1);
            });

    auto linearStatus2Subscriber = nodeHandle->create_subscription<messages::msg::LinearStatus>(
            "linearStatus2", 1,
            [&](const messages::msg::LinearStatus::SharedPtr msg) {
                linearStatusCallback("Linear 2", msg, linear2Counter, linear2);
            });

    auto linearStatus3Subscriber = nodeHandle->create_subscription<messages::msg::LinearStatus>(
            "linearStatus3", 1,
            [&](const messages::msg::LinearStatus::SharedPtr msg) {
                linearStatusCallback("Linear 3", msg, linear3Counter, linear3);
            });

    auto linearStatus4Subscriber = nodeHandle->create_subscription<messages::msg::LinearStatus>(
            "linearStatus4", 1,
            [&](const messages::msg::LinearStatus::SharedPtr msg) {
                linearStatusCallback("Linear 4", msg, linear4Counter, linear4);
            });

    auto zedPositionSubscriber = nodeHandle->create_subscription<messages::msg::ZedPosition>("zed_position",1,zedPositionCallback);
    auto autonomyStatusSubscriber = nodeHandle->create_subscription<messages::msg::AutonomyStatus>("autonomy_status", 10, autonomyStatusCallback);
    auto systemStatusSubscriber = nodeHandle->create_subscription<messages::msg::SystemStatus>("system_status",10,systemStatusCallback);
    auto drivetrainStatusSubscriber = nodeHandle->create_subscription<messages::msg::DrivetrainStatus>("drivetrain_status",10,drivetrainStatusCallback);

    int server_fd, bytesRead; 
    int opt = 1; 
    uint8_t buffer[1024] = {0}; 
    std::string hello("Hello from server");
    std::thread broadcastThread(broadcastIP); //hopefully don't need this anymore
    broadcastThread.detach();

    // Creating socket file descriptor, handling errors
    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0) { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    }
    new_socket = server_fd; //This is the socket that will be used by the other functions above

    // Setting options for socket, handling errors
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 

    // Be open to the client's connection no matter what
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 

    // Bind the socket to the address, handling errors
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 

    bytesRead = recvfrom(server_fd, buffer, 1024, 0, (struct sockaddr *)&address, &addrlen); 
    sendto(server_fd, hello.c_str(), strlen(hello.c_str()), 0, (struct sockaddr *)&address, addrlen); 
    silentRunning=true;
    broadcast=false;
    last_client_tx_time_ms.store(get_time_ms(), std::memory_order_relaxed); 

    fcntl(server_fd, F_SETFL, O_NONBLOCK);


    std::list<uint8_t> messageBytesList;
    uint8_t message[256];
    rclcpp::Rate rate(120);
    bool isClientConnected = true;
    auto previousHeartbeat = std::chrono::high_resolution_clock::now();
    auto previousReset = std::chrono::high_resolution_clock::now();
    
    while(rclcpp::ok()){
        if (!orinLink->is_remote_alive()) {
            RCLCPP_WARN_THROTTLE(nodeHandle->get_logger(), *nodeHandle->get_clock(), 10000, "Remote Dead!");
        }
        last_ros_update_time = get_time_ms();
        try{
            bytesRead = recvfrom(server_fd, buffer, 1024, 0, (struct sockaddr *)&address, &addrlen);
        
            for(int index=0;index<bytesRead;index++){
                messageBytesList.push_back(buffer[index]);
            }
            if(debug){
                if(bytesRead != -1){
                    RCLCPP_INFO(nodeHandle->get_logger(), "Bytes Read: %d", bytesRead);
                }
            }        
        }
        catch(int x){
            RCLCPP_INFO(nodeHandle->get_logger(), "ERROR: Exception when trying to read data from client");
        }

        if(bytesRead > 0){
            if (!isClientConnected) {
                RCLCPP_INFO(nodeHandle->get_logger(), "New client connected. Sending greeting.");
                isClientConnected = true;
                previousHeartbeat = std::chrono::high_resolution_clock::now();
                std::string hello("Hello from server");
                sendto(server_fd, hello.c_str(), hello.length(), 0, (struct sockaddr *)&address, addrlen);
                broadcast = false;
                last_client_tx_time_ms.store(get_time_ms(), std::memory_order_relaxed);
                messageBytesList.clear();
                forceDataResync();
            }
        }
        if (isClientConnected) {
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = now - previousHeartbeat;

            if (elapsed.count() > 5.0) {
                isClientConnected = false;
                RCLCPP_INFO(nodeHandle->get_logger(), "Client disconnected");
                silentRunning = true;
                broadcast = true;
                std::cout << "silentRunning " << silentRunning << std::endl;
            }
            elapsed = now - previousReset;
            if(elapsed.count() > 5){
                forceDataResync();
                previousReset = now;
            }
            uint64_t now_ms = get_time_ms();
            uint64_t last_tx = last_client_tx_time_ms.load(std::memory_order_relaxed);
            if (last_tx == 0) last_client_tx_time_ms.store(now_ms, std::memory_order_relaxed);
            if (now_ms - last_tx > 500) {
                sendClientHeartbeat();
            }
        }

        while(messageBytesList.size()>0 && messageBytesList.front()<=messageBytesList.size()){
	    //RCLCPP_INFO(nodeHandle->get_logger(),"bytes read %d", bytesRead);
            int messageSize=messageBytesList.front();    
            messageBytesList.pop_front();
            messageSize--;
            for(int index=0;index<messageSize;index++){
                message[index]=messageBytesList.front();
                messageBytesList.pop_front();
            }
            //parse command
            // Command values:
            // 0: Heartbeat value
            // 1: Joystick axis values
            // 2: Keystate values
            // 5: Joystick button values
            // 6: Joystick hat values
            // 7: GUI silent running button
            // 8: GUI reboot button

            // TODO: Check if sendRawData == true, send data to Nano
            uint8_t command=message[0];
            if(debug){
                RCLCPP_INFO(nodeHandle->get_logger(), "Message size: %d, Command: %d", messageSize, command);
            }
            if(command==0){
                previousHeartbeat = std::chrono::high_resolution_clock::now();
                if(debug){
                    RCLCPP_INFO(nodeHandle->get_logger(), "Received heartbeat");
                }
            }
            if(command==1){
                messages::msg::AxisState axisState;
                axisState.joystick=message[1];
                axisState.axis=message[2];
                axisState.state=parseFloat(&message[3]);
		        joystickAxisPublisher->publish(axisState);
		        //RCLCPP_INFO(nodeHandle->get_logger(),"axis %d %d %f ", axisState.joystick, axisState.axis , axisState.state);
            }
            if(command==2){
                messages::msg::KeyState keyState;
                keyState.key=((uint16_t)message[1])<<8 | ((uint16_t)message[2]);
                keyState.state=message[3];
                keyPublisher->publish(keyState);
                if(keyState.key == 2){
                    goPublisher->publish(empty);
                }
                if(keyState.key == 49 && keyState.state == 1){
                    return 0;
                }
		        //RCLCPP_INFO(nodeHandle->get_logger(),"key %d %d ", keyState.key , keyState.state);
            }
            if(command==5){
                messages::msg::ButtonState buttonState;
                buttonState.joystick=message[1];
                buttonState.button=message[2];
                buttonState.state=message[3];
                if(buttonState.button==0 && buttonState.state==0){
                    std::cout << "publish stop" << std::endl;
                    stopPublisher->publish(empty);
                }    
                if(buttonState.button==0 && buttonState.state==1){
                    std::cout << "publish go" << std::endl;
                    goPublisher->publish(empty);
                }    
                joystickButtonPublisher->publish(buttonState);
		        //RCLCPP_INFO(nodeHandle->get_logger(),"button %d %d %d", buttonState.joystick , buttonState.button , buttonState.state);
            }
            if(command==6){
                messages::msg::HatState hatState;
                hatState.joystick=message[1];
                hatState.hat=message[2];
                hatState.state=message[3];
                joystickHatPublisher->publish(hatState);
		        //RCLCPP_INFO(nodeHandle->get_logger(),"hat %d %d %d", hatState.joystick , hatState.hat , hatState.state);
            }
            if(command==7){
                silentRunning=message[1];
                std::cout << "silentRunning " << silentRunning << std::endl;
            }
            if(command==8){
                reboot();
                std::cout << "reboot " << silentRunning << std::endl;
            }
        }

        rclcpp::spin_some(nodeHandle);
        commHeartbeatPublisher->publish(heartbeat);
        rate.sleep();
    }

    node_running = false;
    if (comms_thread.joinable()) comms_thread.join();

    broadcastThread.join();
}
