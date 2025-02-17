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

#include <messages/msg/power.hpp>
#include <messages/msg/key_state.hpp>
#include <messages/msg/hat_state.hpp>  
#include <messages/msg/button_state.hpp>
#include <messages/msg/axis_state.hpp>
#include <messages/msg/talon_out.hpp>
#include <messages/msg/zed_position.hpp>
#include <messages/msg/linear_out.hpp>
#include <messages/msg/autonomy_out.hpp>
#include <messages/msg/falcon_out.hpp>

#include <BinaryMessage.hpp>

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
 * \li \b autonomy_out
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

std_msgs::msg::Empty empty;
bool silentRunning=true;
int new_socket;
rclcpp::Node::SharedPtr nodeHandle;
std_msgs::msg::Empty heartbeat;
int total = 0;

int rssi = 0;
#define LOWER_THRESH 67
#define UPPER_THRESH 80
#define CRIT_THRESH 90


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

 
void send(BinaryMessage message){
    //RCLCPP_INFO(nodeHandle->get_logger(), "send message");
    std::shared_ptr<std::list<uint8_t>> byteList = message.getBytes();

    std::vector<uint8_t> bytes(byteList->size());
    int index = 0;
    for(auto byteIterator = byteList->begin(); byteIterator != byteList->end(); byteIterator++, index++){
        bytes.at(index) = *byteIterator;
    }
    if(byteList->size() != 241)
        return;
    try{
        total += byteList->size();
        int bytesSent = 0, byteTotal = 0;
        RCLCPP_INFO(nodeHandle->get_logger(), "sending %s   bytes = %ld", message.getLabel().c_str(), byteList->size());
        while(byteTotal < byteList->size()){
            if((bytesSent = send(new_socket, bytes.data(), byteList->size(), 0))== -1){
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


/*
This function was required to pad the messages to 241 bytes, which was the
size expected by the client to ensure that no bytes were dropped during the
process of being sent. To allow each packet to be 241 bytes, a string with
the name of Pad is added with a string of spaces to fill out the rest. 
Because the packet will have an additional size byte when the total length
of the padded string is over 127, the padded string is split into two when
the size is less than 150.
*/
void pad(BinaryMessage message){
    std::shared_ptr<std::list<uint8_t>> byteList = message.getBytes();
    int size = byteList->size();

    if(size != 241){
        RCLCPP_INFO(nodeHandle->get_logger(), "Received %d bytes", size);
        if(size < 150){
            std::string padded = "";
            for(int i = size; i < size + 50; i++){
                padded.append(" ");
            }
            message.addElementString("Pad", padded);
            size += 58;
        }
        size += 7;
        std::string padded = "";
        for(int i = size; i < 241; i++){
            padded.append(" ");
        }
        message.addElementString("Pad", padded);
    }
    send(message);
}


void send(std::string messageLabel, const messages::msg::FalconOut::SharedPtr talonOut){
    if(silentRunning)return;
    //RCLCPP_INFO(nodeHandle->get_logger(), "send talon");
    BinaryMessage message(messageLabel);

    message.addElementUInt8("Device ID",(uint8_t)talonOut->device_id);
    float volt = talonOut->bus_voltage *= 100.0;
    uint16_t voltage = volt;
    message.addElementUInt16("Bus Voltage",voltage);
    uint16_t current = talonOut->output_current *= 100.0;
    message.addElementUInt16("Output Current",current);
    //message.addElementFloat32("Output Voltage",talonOut->output_voltage);
    message.addElementFloat32("Output Percent",talonOut->output_percent);
    message.addElementUInt8("Temperature",(uint8_t)talonOut->temperature);
    message.addElementUInt16("Sensor Position",(uint8_t)talonOut->sensor_position);
    message.addElementInt8("Sensor Velocity",(uint8_t)talonOut->sensor_velocity);
    message.addElementFloat32("Max Current", talonOut->max_current);
    //message.addElementBoolean("Temp Disable", talonOut->temp_disable);
    //message.addElementBoolean("Volt Disable", talonOut->volt_disable);

    pad(message);
}


void send(std::string messageLabel, const messages::msg::TalonOut::SharedPtr talonOut){
    if(silentRunning)return;
    //RCLCPP_INFO(nodeHandle->get_logger(), "send talon");
    BinaryMessage message(messageLabel);

    message.addElementInt8("Device ID",talonOut->device_id);
    float volt = talonOut->bus_voltage *= 100.0;
    uint16_t voltage = volt;
    message.addElementUInt16("Bus Voltage",voltage);
    uint16_t current = talonOut->output_current *= 100.0;
    message.addElementUInt16("Output Current",current);
    //message.addElementFloat32("Output Voltage",talonOut->output_voltage);
    message.addElementFloat32("Output Percent",talonOut->output_percent);
    message.addElementUInt8("Temperature",(uint8_t)talonOut->temperature);
    message.addElementUInt16("Sensor Position",talonOut->sensor_position);
    message.addElementInt8("Sensor Velocity",(int8_t)talonOut->sensor_velocity);
    message.addElementFloat32("Max Current", talonOut->max_current);
    //message.addElementBoolean("Temp Disable", talonOut->temp_disable);
    //message.addElementBoolean("Volt Disable", talonOut->volt_disable);
    pad(message);
}


void send(std::string messageLabel, const messages::msg::Power::SharedPtr power){
    if(silentRunning)return;
    //RCLCPP_INFO(nodeHandle->get_logger(), "send power");
    BinaryMessage message(messageLabel);

    message.addElementFloat32("Voltage",power->voltage);
    message.addElementFloat32("Current 0",power->current0);
    message.addElementFloat32("Current 1",power->current1);
    message.addElementFloat32("Current 2",power->current2);
    message.addElementFloat32("Current 3",power->current3);
    message.addElementFloat32("Current 4",power->current4);
    message.addElementFloat32("Current 5",power->current5);
    message.addElementFloat32("Current 6",power->current6);
    message.addElementFloat32("Current 7",power->current7);
    message.addElementFloat32("Current 8",power->current8);
    message.addElementFloat32("Current 9",power->current9);
    message.addElementFloat32("Current 10",power->current10);
    message.addElementFloat32("Current 11",power->current11);
    message.addElementFloat32("Current 12",power->current12);
    message.addElementFloat32("Current 13",power->current13);
    message.addElementFloat32("Current 14",power->current14);
    message.addElementFloat32("Current 15",power->current15);

    //pad(message);
}


void send(std::string messageLabel, const messages::msg::LinearOut::SharedPtr linear){
    if(silentRunning)return;

    BinaryMessage message(messageLabel);

    message.addElementUInt8("Motor Number", (uint8_t)linear->motor_number);
    message.addElementFloat32("Speed", linear->speed);
    message.addElementUInt16("Potentiometer", (uint16_t)linear->potentiometer);
    message.addElementUInt8("Time Without Change", (uint8_t)linear->time_without_change);
    message.addElementUInt16("Max", (uint16_t)linear->max);
    message.addElementUInt16("Min", (uint16_t)linear->min);
    message.addElementString("Error", linear->error);
    message.addElementBoolean("At Min", linear->at_min);
    message.addElementBoolean("At Max", linear->at_max);
    message.addElementFloat32("Distance", linear->distance);
    message.addElementBoolean("Sensorless", linear->sensorless);

    pad(message);
}


void send(std::string messageLabel, const messages::msg::AutonomyOut::SharedPtr autonomy){
    if(silentRunning)return;

    BinaryMessage message(messageLabel);
    message.addElementString("Robot State", autonomy->robot_state);
    message.addElementString("Excavation State", autonomy->excavation_state);
    message.addElementString("Error State", autonomy->error_state);
    message.addElementString("Diagnostics State", autonomy->diagnostics_state);
    
    pad(message);
}


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
    BinaryMessage message("Zed");
    message.addElementFloat32("X", zedPosition->x);
    message.addElementFloat32("Y", zedPosition->y);
    message.addElementFloat32("Z", zedPosition->z);
    message.addElementFloat32("roll", zedPosition->roll);
    message.addElementFloat32("pitch", zedPosition->pitch);
    message.addElementFloat32("yaw", zedPosition->yaw);
    message.addElementFloat32("aruco roll", zedPosition->aruco_roll);
    message.addElementFloat32("aruco pitch", zedPosition->aruco_pitch);
    message.addElementFloat32("aruco yaw", zedPosition->aruco_yaw);
    message.addElementBoolean("aruco", zedPosition->aruco_visible);
    pad(message);
}


void communicationCallback(){
    if(silentRunning)return;
    BinaryMessage message("Communication");
    if(rssi < LOWER_THRESH)
        message.addElementString("Wi-Fi", "NORMAL OPERATION");
    else if(rssi >= LOWER_THRESH && rssi < UPPER_THRESH)
        message.addElementString("Wi-Fi", "DEGRADED OPERATION");
    else if(rssi >= UPPER_THRESH && rssi < CRIT_THRESH)
        message.addElementString("Wi-Fi", "SEVERE INTERFERENCE OPERATION");
    else
        message.addElementString("Wi-Fi", "NON-FUNCIONAL OPERATION");
    message.addElementString("CAN Bus", "NON-FUNCTIONAL OPERATION");
    pad(message);
}


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
    if(rssi < UPPER_THRESH)
        send("Power",power);
}

/** @brief Callback function for the Talon topic
 * 
 * This function receives the talonOut message published by the first 
 * Talon and uses the send function to send the data to the client side
 * GUI.
 * @param talonOut
 * @return void
 * */
void talon1Callback(const messages::msg::TalonOut::SharedPtr talonOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "talon1 callback");
    if(rssi < CRIT_THRESH)
        send("Talon 1",talonOut);
}

/** @brief Callback function for the Talon topic
 * 
 * This function receives the talonOut message published by the first 
 * Talon and uses the send function to send the data to the client side
 * GUI.
 * @param talonOut
 * @return void
 * */
void talon2Callback(const messages::msg::TalonOut::SharedPtr talonOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "talon2 callback");
    if(rssi < CRIT_THRESH)
        send("Talon 2",talonOut);
}

/** @brief Callback function for the Talon topic
 * 
 * This function receives the talonOut message published by the first 
 * Talon and uses the send function to send the data to the client side
 * GUI.
 * @param talonOut
 * @return void
 * */
void talon3Callback(const messages::msg::TalonOut::SharedPtr talonOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "talon3 callback");
    if(rssi < CRIT_THRESH)
        send("Talon 3",talonOut);
}

/** @brief Callback function for the Talon topic
 * 
 * This function receives the talonOut message published by the first 
 * Talon and uses the send function to send the data to the client side
 * GUI.
 * @param talonOut
 * @return void
 * */
void talon4Callback(const messages::msg::TalonOut::SharedPtr talonOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "talon4 callback");
    if(rssi < CRIT_THRESH)
        send("Talon 4",talonOut);
}


/** @brief Callback function for the Talon topic
 * 
 * This function receives the talonOut message published by the first 
 * Talon and uses the send function to send the data to the client side
 * GUI.
 * @param talonOut
 * @return void
 * */
void falcon1Callback(const messages::msg::FalconOut::SharedPtr talonOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "falcon1 callback");
    if(rssi < CRIT_THRESH)
        send("Falcon 1",talonOut);
}

/** @brief Callback function for the Talon topic
 * 
 * This function receives the talonOut message published by the first 
 * Talon and uses the send function to send the data to the client side
 * GUI.
 * @param talonOut
 * @return void
 * */
void falcon2Callback(const messages::msg::FalconOut::SharedPtr talonOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "falcon2 callback");
    if(rssi < CRIT_THRESH)
        send("Falcon 2",talonOut);
}

/** @brief Callback function for the Talon topic
 * 
 * This function receives the talonOut message published by the first 
 * Talon and uses the send function to send the data to the client side
 * GUI.
 * @param talonOut
 * @return void
 * */
void falcon3Callback(const messages::msg::FalconOut::SharedPtr talonOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "falcon3 callback");
    if(rssi < CRIT_THRESH)
        send("Falcon 3",talonOut);
}

/** @brief Callback function for the Talon topic
 * 
 * This function receives the talonOut message published by the first 
 * Talon and uses the send function to send the data to the client side
 * GUI.
 * @param talonOut
 * @return void
 * */
void falcon4Callback(const messages::msg::FalconOut::SharedPtr talonOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "falcon4 callback");
    if(rssi < CRIT_THRESH)
        send("Falcon 4",talonOut);
}


/** @brief Callback function for the LinearOut topic
 * 
 * This function receives the linearOut message published by the excavation
 * node and uses the send data to send the data to the client side GUI.
 * @param linearOut 
 */
void linearOut1Callback(const messages::msg::LinearOut::SharedPtr linearOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "linear1 callback");
    if(rssi < UPPER_THRESH)
        send("Linear 1", linearOut);
}


/** @brief Callback function for the LinearOut topic
 * 
 * This function receives the linearOut message published by the excavation
 * node and uses the send data to send the data to the client side GUI.
 * @param linearOut 
 */
void linearOut2Callback(const messages::msg::LinearOut::SharedPtr linearOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "linear2 callback");
    if(rssi < UPPER_THRESH)
        send("Linear 2", linearOut);
}


/** @brief Callback function for the LinearOut topic
 * 
 * This function receives the linearOut message published by the excavation
 * node and uses the send data to send the data to the client side GUI.
 * @param linearOut 
 */
void linearOut3Callback(const messages::msg::LinearOut::SharedPtr linearOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "linear3 callback");
    if(rssi < UPPER_THRESH)
        send("Linear 3", linearOut);
}


/** @brief Callback function for the LinearOut topic
 * 
 * This function receives the linearOut message published by the excavation
 * node and uses the send data to send the data to the client side GUI.
 * @param linearOut 
 */
void linearOut4Callback(const messages::msg::LinearOut::SharedPtr linearOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "linear4 callback");
    if(rssi < UPPER_THRESH)
        send("Linear 4", linearOut);
}


void autonomyOutCallback(const messages::msg::AutonomyOut::SharedPtr autonomyOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "autonomy callback");
    if(rssi < CRIT_THRESH)
        send("Autonomy", autonomyOut);
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
            std::string addressString=getAddressString(AF_INET,"wlP1p1s0");

            std::string message(robotName+"@"+addressString);
            std::cout << message << std::endl << std::flush;

            int socketDescriptor=socket(AF_INET, SOCK_DGRAM, 0);

            //if(socket>=0){
            if(socketDescriptor>=0){
                struct sockaddr_in socketAddress;
                socketAddress.sin_family=AF_INET;
                socketAddress.sin_addr.s_addr = inet_addr("226.1.1.1");
                socketAddress.sin_port = htons(4321);

                struct in_addr localInterface;
                localInterface.s_addr = inet_addr(addressString.c_str());
                if(setsockopt(socketDescriptor, IPPROTO_IP, IP_MULTICAST_IF, (char*)&localInterface, sizeof(localInterface))>=0){
                    sendto(socketDescriptor,message.c_str(),message.length(),0,(struct sockaddr*)&socketAddress, sizeof(socketAddress));
                }
            }
            close(socketDescriptor);
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);

    nodeHandle = rclcpp::Node::make_shared("communication2");
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting communication2 node");

    nodeHandle->declare_parameter<std::string>("robot_name","not named");
    rclcpp::Parameter robotNameParameter = nodeHandle->get_parameter("robot_name");
    robotName = robotNameParameter.as_string();
    RCLCPP_INFO(nodeHandle->get_logger(),"robotName: %s", robotName.c_str());

    auto joystickAxisPublisher = nodeHandle->create_publisher<messages::msg::AxisState>("joystick_axis", 1);
    auto joystickHatPublisher = nodeHandle->create_publisher<messages::msg::HatState>("joystick_hat",1);
    auto joystickButtonPublisher = nodeHandle->create_publisher<messages::msg::ButtonState>("joystick_button",1);
    auto keyPublisher = nodeHandle->create_publisher<messages::msg::KeyState>("key",1);
    auto stopPublisher = nodeHandle->create_publisher<std_msgs::msg::Empty>("STOP",1);
    auto goPublisher=nodeHandle->create_publisher<std_msgs::msg::Empty>("GO",1);
    auto commHeartbeatPublisher = nodeHandle->create_publisher<std_msgs::msg::Empty>("comm_heartbeat",1);

    auto powerSubscriber = nodeHandle->create_subscription<messages::msg::Power>("power",1,powerCallback);
    auto talon1Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_14_info",1,talon1Callback);
    auto talon2Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_15_info",1,talon2Callback);
    auto talon3Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_16_info",1,talon3Callback);
    auto talon4Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_17_info",1,talon4Callback);
    auto falcon1Subscriber = nodeHandle->create_subscription<messages::msg::FalconOut>("talon_10_info",1,falcon1Callback);
    auto falcon2Subscriber = nodeHandle->create_subscription<messages::msg::FalconOut>("talon_11_info",1,falcon2Callback);
    auto falcon3Subscriber = nodeHandle->create_subscription<messages::msg::FalconOut>("talon_12_info",1,falcon3Callback);
    auto falcon4Subscriber = nodeHandle->create_subscription<messages::msg::FalconOut>("talon_13_info",1,falcon4Callback);
    auto linearOut1Subscriber = nodeHandle->create_subscription<messages::msg::LinearOut>("linearOut1",1,linearOut1Callback);
    auto linearOut2Subscriber = nodeHandle->create_subscription<messages::msg::LinearOut>("linearOut2",1,linearOut2Callback);
    auto linearOut3Subscriber = nodeHandle->create_subscription<messages::msg::LinearOut>("linearOut3",1,linearOut3Callback);
    auto linearOut4Subscriber = nodeHandle->create_subscription<messages::msg::LinearOut>("linearOut4",1,linearOut4Callback);
    auto zedPositionSubscriber = nodeHandle->create_subscription<messages::msg::ZedPosition>("zed_position",1,zedPositionCallback);
    auto autonomyOutSubscriber = nodeHandle->create_subscription<messages::msg::AutonomyOut>("autonomy_out", 10, autonomyOutCallback);

    int server_fd, bytesRead; 
    struct sockaddr_in address; 
    int opt = 1; 
    int addrlen = sizeof(address); 
    uint8_t buffer[1024] = {0}; 
    std::string hello("Hello from server");

    std::string result = "";
    char buffer2[128];


    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    }
    std::thread broadcastThread(broadcastIP);

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    if (listen(server_fd, 3) < 0) { 
        perror("listen"); 
        exit(EXIT_FAILURE); 
    } 
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) { 
        perror("accept"); 
        exit(EXIT_FAILURE); 
    }

    broadcast=false;
    bytesRead = read(new_socket, buffer, 1024); 
    send(new_socket, hello.c_str(), strlen(hello.c_str()), 0); 
    silentRunning=true;

    fcntl(new_socket, F_SETFL, O_NONBLOCK);
    

    std::list<uint8_t> messageBytesList;
    uint8_t message[256];
    rclcpp::Rate rate(30);
    while(rclcpp::ok()){
        try{
            bytesRead = recv(new_socket, buffer, 1024, 0);
            for(int index=0;index<bytesRead;index++){
                messageBytesList.push_back(buffer[index]);
            }

            if(bytesRead==0){
                stopPublisher->publish(empty);
                RCLCPP_INFO(nodeHandle->get_logger(),"Lost Connection");
                broadcast=true;
                //wait for reconnect
                if (listen(server_fd, 3) < 0) { 
                    perror("listen"); 
                    exit(EXIT_FAILURE); 
                } 
                if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) { 
                    perror("accept"); 
                    exit(EXIT_FAILURE); 
                }
                broadcast=false;
                bytesRead = read(new_socket, buffer, 1024); 
                send(new_socket, hello.c_str(), strlen(hello.c_str()), 0); 
                fcntl(new_socket, F_SETFL, O_NONBLOCK);
        
                silentRunning=true;
            }
        }
        catch(int x){
            RCLCPP_INFO(nodeHandle->get_logger(), "ERROR: Exception when trying to read data from client");
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
            // 1: Joystick axis values
            // 2: Keystate values
            // 5: Joystick button values
            // 6: Joystick hat values
            // 7: GUI silent running button
            // 8: GUI reboot button
            uint8_t command=message[0];
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

        FILE* pipe = popen("iwconfig wlP1p1s0 | grep -E -o '=-.{0,2}'", "r");
        while(!feof(pipe)){
            if(fgets(buffer2, 128, pipe) != nullptr){
                result += buffer2;
            }
        }
        rssi = ((int)result[2] - 48 ) * 10 + ((int)result[3] - 48);
        pclose(pipe);
        communicationCallback();
        rate.sleep();
    }

    broadcastThread.join();
}
