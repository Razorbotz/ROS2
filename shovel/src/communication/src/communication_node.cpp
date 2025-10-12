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
#include "utils/utils.hpp"

#define PORT 31337

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

#define LOWER_THRESH 67
#define UPPER_THRESH 80
#define CRIT_THRESH 90

struct Falcon {
    uint8_t device_id;
    uint16_t voltage;
    uint16_t current;
    float output_percent;
    uint8_t temperature;
    float sensor_position;
    float sensor_velocity;
    float max_current;
    bool temp_disable;
    bool error;
};

Falcon falcon1, falcon2, falcon3, falcon4;

struct Talon {
    uint8_t device_id;
    uint16_t voltage;
    uint16_t current;
    float output_percent;
    uint8_t temperature;
    float sensor_position;
    float sensor_velocity;
    float max_current;
    bool temp_disable;
};

Talon talon1, talon2, talon3, talon4;

struct Linear {
    uint8_t motor_number;
    float speed;
    uint16_t potentiometer;
    uint8_t time_without_change;
    uint16_t max;
    uint16_t min;
    std::string error;
    bool at_min;
    bool at_max;
    float distance;
    bool sensorless;
};

Linear linear1, linear2, linear3, linear4;


struct AutonomyState {
    std::string robot_state;
    std::string excavation_state;
    std::string error_state;
    std::string diagnostics_state;
    std::string tilt_state;
    std::string dump_state;
    std::string bucket_state;
    std::string arms_state;
    float dest_x = 0.0f;
    float dest_z = 0.0f;
};

AutonomyState autonomyState;


struct ZedState {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    bool aruco;
};

ZedState zedState;

struct DrivetrainState {
    float f1_vel;
    float f1_rpm;
    float f1_speed;
    float f2_vel;
    float f2_rpm;
    float f2_speed;
    float f3_vel;
    float f3_rpm;
    float f3_speed;
    float f4_vel;
    float f4_rpm;
    float f4_speed;
};

DrivetrainState drivetrainState;

struct SystemState {
    int32_t  rssi;
    std::string wifi;
    std::string can_bus;
    bool using_can1;
    int32_t  rx_packets;
    int32_t  tx_packets;
    std::string can_bus2;
    int32_t  rx_packets2;
    int32_t  tx_packets2;
    int32_t  first_motor;
    int32_t  second_motor;
    int32_t  num_breaks;
};

SystemState systemState;

/** @brief Parse a byte represenation into a float.
 * 
 * @param array
 * @return value
 * */
float parseFloat(uint8_t* array){
    uint32_t axisYInteger=0;
    axisYInteger|=uint32_t(array[0])<<24;    
    axisYInteger|=uint32_t(array[1])<<16;    
    axisYInteger|=uint32_t(array[2])<<8;    
    axisYInteger|=uint32_t(array[3])<<0;    
    float value=(float)*(static_cast<float*>(static_cast<void*>(&axisYInteger)));

    return value;
}

int key = 0x2C;
void checksum_encode(std::shared_ptr<std::list<uint8_t>> byteList){
    uint32_t sum = 0;  // Use a wider type to avoid overflow

    // Append zero byte as placeholders for the checksum
    byteList->push_back(0x00);


    //std::cout << "Bytes with placeholders: ";
    // for (auto byte : *byteList) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    // }
    //std::cout << std::endl;

    // Sum all the bytes
    for (uint8_t byte : *byteList) {
        sum += byte;
    }

    // Compute Checksum
    uint8_t checksum = sum % key;
    //std::cout << "Simple checksum computed: 0x" << std::hex << static_cast<int>(checksum) << std::endl;

    
    auto it = byteList->end();
    std::advance(it, -1);
    *it = checksum;

    // std::cout << "Final byteList: ";
    // for (auto byte : *byteList) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    // }
    // std::cout << std::endl;
}

 
void send(BinaryMessage message){
    //RCLCPP_INFO(nodeHandle->get_logger(), "send message");
    std::shared_ptr<std::list<uint8_t>> byteList = message.getBytes();
    checksum_encode(byteList);    

    std::vector<uint8_t> bytes(byteList->size());
    int index = 0;
    for(auto byteIterator = byteList->begin(); byteIterator != byteList->end(); byteIterator++, index++){
        bytes.at(index) = *byteIterator;
    }
    //if(byteList->size() != 242)
    //    return;
    try{
        total += byteList->size();
        int bytesSent = 0, byteTotal = 0;
        //RCLCPP_INFO(nodeHandle->get_logger(), "sending %s   bytes = %ld", message.getLabel().c_str(), byteList->size());
        while(byteTotal < byteList->size()){
            if((bytesSent = sendto(new_socket, bytes.data(), byteList->size(), 0, (struct sockaddr *)&address, addrlen))== -1){
                RCLCPP_INFO(nodeHandle->get_logger(), "Failed to send message.");   
                break;
            }
            else{
                byteTotal += bytesSent;
            }
        }
    }
    catch(int x){
        RCLCPP_INFO(nodeHandle->get_logger(), "ERROR: Exception when trying to send data to client");
    }

}

void update_if_changed(BinaryMessage& msg, bool& changed, uint8_t& old_val, uint8_t new_val, const std::string& label) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementUInt8(label, new_val);
        old_val = new_val;
        RCLCPP_INFO(nodeHandle->get_logger(), "Added '%s' to message", label.c_str());
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, std::string& old_val, const std::string& new_val, const std::string& label) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementString(label, new_val);
        old_val = new_val;
        RCLCPP_INFO(nodeHandle->get_logger(), "Added '%s' to message", label.c_str());
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, uint16_t& old_val, uint16_t new_val, const std::string& label) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementUInt16(label, new_val);
        old_val = new_val;
        RCLCPP_INFO(nodeHandle->get_logger(), "Added '%s' to message", label.c_str());
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, float& old_val, float new_val, const std::string& label) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementFloat32(label, new_val);
        old_val = new_val;
        RCLCPP_INFO(nodeHandle->get_logger(), "Added '%s' to message", label.c_str());
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, bool& old_val, bool new_val, const std::string& label) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementBoolean(label, new_val);
        old_val = new_val;
        RCLCPP_INFO(nodeHandle->get_logger(), "Added '%s' to message", label.c_str());
    }
}

void update_if_changed(BinaryMessage& msg, bool& changed, int& old_val, int new_val, const std::string& label) {
    if (old_val != new_val) {
        changed = true;
        msg.addElementInt32(label, new_val);
        old_val = new_val;
        RCLCPP_INFO(nodeHandle->get_logger(), "Added '%s' to message", label.c_str());
    }
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


float voltage = 0.0f;
float temperature = 0.0f;
std::array<float, 16> currents{};

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
    //RCLCPP_INFO(nodeHandle->get_logger(), "autonomy callback");
    autonomyCounter++;
    if(autonomyCounter % 15 == 0)
        if(rssi < CRIT_THRESH)
            send("Autonomy", autonomyStatus);
}


/** @brief Returns the address string of the rover.
 * 
 * This function is called when the node
 * tries to setup the socket connection between the rover and client. This function
 * returns the address as a string.
 * @param family
 * @param interfaceName
 * @return addressString
 * */
std::string getAddressString(int family, std::string interfaceName){
    std::string addressString("");
    ifaddrs* interfaceAddresses = nullptr;
    for (int failed=getifaddrs(&interfaceAddresses); !failed && interfaceAddresses; interfaceAddresses=interfaceAddresses->ifa_next){
        if(strcmp(interfaceAddresses->ifa_name,interfaceName.c_str())==0 && interfaceAddresses->ifa_addr->sa_family == family) {
            if (interfaceAddresses->ifa_addr->sa_family == AF_INET) {
                sockaddr_in *socketAddress = reinterpret_cast<sockaddr_in *>(interfaceAddresses->ifa_addr);
                addressString += inet_ntoa(socketAddress->sin_addr);
            }
            if (interfaceAddresses->ifa_addr->sa_family == AF_INET6) {
                sockaddr_in6 *socketAddress = reinterpret_cast<sockaddr_in6 *>(interfaceAddresses->ifa_addr);
                for (int index = 0; index < 16; index += 2) {
                    char bits[5];
                    sprintf(bits,"%02x%02x", socketAddress->sin6_addr.s6_addr[index],socketAddress->sin6_addr.s6_addr[index + 1]);
                    if (index)addressString +=":";
                    addressString +=bits;
                }
            }
            if (interfaceAddresses->ifa_addr->sa_family == AF_PACKET) {
                sockaddr_ll *socketAddress = reinterpret_cast<sockaddr_ll *>(interfaceAddresses->ifa_addr);
                for (int index = 0; index < socketAddress->sll_halen; index++) {
                    char bits[3];
                    sprintf(bits,"%02x", socketAddress->sll_addr[index]);
                    if (index)addressString +=":";
                    addressString +=bits;
                }
            }
        }
    }
    freeifaddrs(interfaceAddresses);
    return addressString;
}


/** @brief Prints the address
 * 
 * */
void printAddresses() {
    printf("Addresses\n");
    ifaddrs* interfaceAddresses = nullptr;
    for (int failed=getifaddrs(&interfaceAddresses); !failed && interfaceAddresses; interfaceAddresses=interfaceAddresses->ifa_next){
        printf("%s ",interfaceAddresses->ifa_name);
        if(interfaceAddresses->ifa_addr->sa_family == AF_INET){
            printf("AF_INET ");
            sockaddr_in* socketAddress=reinterpret_cast<sockaddr_in*>(interfaceAddresses->ifa_addr);
            printf("%d ",socketAddress->sin_port);
            printf("%s ",inet_ntoa(socketAddress->sin_addr));
        }
        if(interfaceAddresses->ifa_addr->sa_family == AF_INET6){
            printf("AF_INET6 ");
            sockaddr_in6* socketAddress=reinterpret_cast<sockaddr_in6*>(interfaceAddresses->ifa_addr);
            printf("%d ",socketAddress->sin6_port);
            printf("%d ",socketAddress->sin6_flowinfo); 
            for(int index=0;index<16;index+=2) {
                if(index)printf(":");
                printf("%02x%02x",socketAddress->sin6_addr.s6_addr[index],socketAddress->sin6_addr.s6_addr[index+1]);
            }
        }
        if(interfaceAddresses->ifa_addr->sa_family == AF_PACKET){
            printf("AF_PACKET ");
            sockaddr_ll* socketAddress=reinterpret_cast<sockaddr_ll*>(interfaceAddresses->ifa_addr);
            printf("%d ",socketAddress->sll_protocol);
            printf("%d ",socketAddress->sll_ifindex);
            printf("%d ",socketAddress->sll_hatype);
            printf("%d ",socketAddress->sll_pkttype);
            for(int index=0;index<socketAddress->sll_halen;index++){
                if(index)printf(":");
                printf("%02x",socketAddress->sll_addr[index]);
            }
        }
        printf("\n");
    }
    printf("Done\n");
}


/** @brief Reboots the rover. 
 *
 * */
void reboot(){
    sync();
    reboot(LINUX_REBOOT_CMD_POWER_OFF);
}

std::string robotName="unnamed";
std::string interfaceName = "wlan0";
bool broadcast=true;


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
            std::cout << message << std::endl << std::flush;

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


int main(int argc, char **argv){
    rclcpp::init(argc,argv);

    nodeHandle = rclcpp::Node::make_shared("communication");
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting communication node");

    robotName = utils::getParameter<std::string>(nodeHandle, "robot_name", "not named");
    debug = utils::getParameter<bool>(nodeHandle, "debug", false);

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

    fcntl(server_fd, F_SETFL, O_NONBLOCK);
    

    std::list<uint8_t> messageBytesList;
    uint8_t message[256];
    rclcpp::Rate rate(90);
    bool isClientConnected = true;
    auto previousHeartbeat = std::chrono::high_resolution_clock::now();
    
    while(rclcpp::ok()){
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
                messageBytesList.clear();
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

    broadcastThread.join(); //hopefully don't need this anymore
}
