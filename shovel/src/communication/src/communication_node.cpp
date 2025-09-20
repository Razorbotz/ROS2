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
#include <messages/msg/talon_status.hpp>
#include <messages/msg/zed_position.hpp>
#include <messages/msg/linear_status.hpp>
#include <messages/msg/autonomy_status.hpp>
#include <messages/msg/falcon_status.hpp>
#include <messages/msg/system_status.hpp>

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


void send(std::string messageLabel, const messages::msg::FalconStatus::SharedPtr talonStatus){
    if(silentRunning)return;
    //RCLCPP_INFO(nodeHandle->get_logger(), "send talon");
    BinaryMessage message(messageLabel);

    message.addElementUInt8("Device ID",(uint8_t)talonStatus->device_id);
    float volt = talonStatus->bus_voltage *= 100.0;
    uint16_t voltage = volt;
    message.addElementUInt16("Bus Voltage",voltage);
    uint16_t current = talonStatus->output_current *= 100.0;
    message.addElementUInt16("Output Current",current);
    //message.addElementFloat32("Output Voltage",talonStatus->output_voltage);
    message.addElementFloat32("Output Percent",talonStatus->output_percent);
    message.addElementUInt8("Temperature",(uint8_t)talonStatus->temperature);
    message.addElementUInt16("Sensor Position",(uint8_t)talonStatus->sensor_position);
    message.addElementInt8("Sensor Velocity",(uint8_t)talonStatus->sensor_velocity);
    message.addElementFloat32("Max Current", talonStatus->max_current);
    message.addElementBoolean("Temp Disable", talonStatus->temp_disable);
    message.addElementBoolean("Error", talonStatus->error);

    send(message);
}


void send(std::string messageLabel, const messages::msg::TalonStatus::SharedPtr talonStatus){
    if(silentRunning)return;
    //RCLCPP_INFO(nodeHandle->get_logger(), "send talon");
    BinaryMessage message(messageLabel);

    message.addElementInt8("Device ID",talonStatus->device_id);
    float volt = talonStatus->bus_voltage *= 100.0;
    uint16_t voltage = volt;
    message.addElementUInt16("Bus Voltage",voltage);
    uint16_t current = talonStatus->output_current *= 100.0;
    message.addElementUInt16("Output Current",current);
    //message.addElementFloat32("Output Voltage",talonStatus->output_voltage);
    message.addElementFloat32("Output Percent",talonStatus->output_percent);
    message.addElementUInt8("Temperature",(uint8_t)talonStatus->temperature);
    message.addElementUInt16("Sensor Position",talonStatus->sensor_position);
    message.addElementInt8("Sensor Velocity",(int8_t)talonStatus->sensor_velocity);
    message.addElementFloat32("Max Current", talonStatus->max_current);
    message.addElementBoolean("Temp Disable", talonStatus->temp_disable);
    send(message);
}


void send(std::string messageLabel, const messages::msg::Power::SharedPtr power){
    if(silentRunning)return;
    //RCLCPP_INFO(nodeHandle->get_logger(), "send power");
    BinaryMessage message(messageLabel);

    message.addElementFloat32("Voltage",power->voltage);
    message.addElementFloat32("Temp",power->temperature);
    message.addElementFloat32("Current 0",power->current0);
    message.addElementFloat32("Current 1",power->current1);
    message.addElementFloat32("Current 2",power->current2);
    message.addElementFloat32("Current 3",power->current3);
    message.addElementFloat32("Current 4",power->current4);
    message.addElementFloat32("Current 5",power->current5);
    message.addElementFloat32("Current 6",power->current6);
    
    BinaryMessage message2("Power2");
    message2.addElementFloat32("Current 7",power->current7);
    message2.addElementFloat32("Current 8",power->current8);
    message2.addElementFloat32("Current 9",power->current9);
    message2.addElementFloat32("Current 10",power->current10);
    message2.addElementFloat32("Current 11",power->current11);
    message2.addElementFloat32("Current 12",power->current12);
    message2.addElementFloat32("Current 13",power->current13);
    message2.addElementFloat32("Current 14",power->current14);
    message2.addElementFloat32("Current 15",power->current15);

    send(message);
    send(message2);
}


void send(std::string messageLabel, const messages::msg::LinearStatus::SharedPtr linear){
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

    send(message);
}


void send(std::string messageLabel, const messages::msg::AutonomyStatus::SharedPtr autonomy){
    if(silentRunning)return;

    BinaryMessage message(messageLabel);
    message.addElementString("Robot State", autonomy->robot_state);
    message.addElementString("Excavation State", autonomy->excavation_state);
    message.addElementString("Error State", autonomy->error_state);
    message.addElementString("Diagnostics State", autonomy->diagnostics_state);
    message.addElementString("Tilt State", autonomy->tilt_state);
    message.addElementString("Dump State", autonomy->dump_state);
    message.addElementString("Level Bucket", autonomy->bucket_state);
    message.addElementString("Level Arms", autonomy->arms_state);
    message.addElementFloat32("Dest X", autonomy->dest_x);
    message.addElementFloat32("Dest Z", autonomy->dest_z);

    send(message);
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
    message.addElementBoolean("aruco", zedPosition->aruco_visible);
    send(message);
}


void systemStatusCallback(const messages::msg::SystemStatus::SharedPtr status){
    if(silentRunning)return;
    BinaryMessage message("Communication");
    rssi = status->rssi;
    message.addElementInt32("RSSI", rssi);
    if(rssi < LOWER_THRESH)
        message.addElementString("Wi-Fi", "NORMAL");
    else if(rssi >= LOWER_THRESH && rssi < UPPER_THRESH)
        message.addElementString("Wi-Fi", "DEGRADED");
    else if(rssi >= UPPER_THRESH && rssi < CRIT_THRESH)
        message.addElementString("Wi-Fi", "INTERFERENCE");
    else
        message.addElementString("Wi-Fi", "NON-FUNCIONAL");
    message.addElementString("CAN Bus", status->can_message);
    usingCAN1 = status->using_can1;
    message.addElementBoolean("Using CAN1", usingCAN1);
    message.addElementInt32("RX packets", status->rx_packets);
    message.addElementInt32("TX packets", status->tx_packets);
    message.addElementString("CAN Bus2", status->can2_message);
    message.addElementInt32("RX2 packets", status->rx2_packets);
    message.addElementInt32("TX2 packets", status->tx2_packets);
    message.addElementInt32("First Motor", status->first_motor);
    message.addElementInt32("Second Motor", status->second_motor);
    message.addElementInt32("Num Breaks", status->num_breaks);
    send(message);
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
 * This function receives the talonStatus message published by the first 
 * Talon and uses the send function to send the data to the client side
 * GUI.
 * @param talonStatus
 * @return void
 * */
void talonStatusCallback(const std::string& name, const messages::msg::TalonStatus::SharedPtr talonStatus){
    //RCLCPP_INFO(nodeHandle->get_logger(), "talon1 callback");
    if(rssi < CRIT_THRESH)
        send(name, talonStatus);
}


/** @brief Callback function for the Talon topic
 * 
 * This function receives the talonStatus message published by the first 
 * Talon and uses the send function to send the data to the client side
 * GUI.
 * @param talonStatus
 * @return void
 * */
void falconStatusCallback(const std::string& name, const messages::msg::FalconStatus::SharedPtr talonStatus){
    //RCLCPP_INFO(nodeHandle->get_logger(), "falcon1 callback");
    if(rssi < CRIT_THRESH)
        send(name,talonStatus);
}


/** @brief Callback function for the LinearStatus topic
 * 
 * This function receives the linearStatus message published by the excavation
 * node and uses the send data to send the data to the client side GUI.
 * @param name
 * @param linearStatus 
 */
void linearStatusCallback(const std::string& name, const messages::msg::LinearStatus::SharedPtr linearStatus){
    //RCLCPP_INFO(nodeHandle->get_logger(), "%s callback", name.c_str());
    if(rssi < UPPER_THRESH)
        send(name, linearStatus);
}


void autonomyStatusCallback(const messages::msg::AutonomyStatus::SharedPtr autonomyStatus){
    //RCLCPP_INFO(nodeHandle->get_logger(), "autonomy callback");
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
// bool broadcast=true;


/** @brief Creates socketDescriptor for socket connection.
 * 
 * This function is called when the node
 * tries to setup the socket connection between the rover and client.
 * This function creates the socketDescriptor for the socket connection.
 * Uses the getAddressString function.
 * */
// void broadcastIP(){
//     while(true){
//         if(broadcast){
//             std::string addressString=getAddressString(AF_INET,interfaceName);

//             std::string message(robotName+"@"+addressString);
//             std::cout << message << std::endl << std::flush;

//             int socketDescriptor=socket(AF_INET, SOCK_DGRAM, 0);

//             //if(socket>=0){
//             if(socketDescriptor>=0){
//                 struct sockaddr_in socketAddress;
//                 socketAddress.sin_family=AF_INET;
//                 socketAddress.sin_addr.s_addr = inet_addr("226.1.1.1");
//                 socketAddress.sin_port = htons(4321);

//                 struct in_addr localInterface;
//                 localInterface.s_addr = inet_addr(addressString.c_str());
//                 if(setsockopt(socketDescriptor, IPPROTO_IP, IP_MULTICAST_IF, (char*)&localInterface, sizeof(localInterface))>=0){
//                     sendto(socketDescriptor,message.c_str(),message.length(),0,(struct sockaddr*)&socketAddress, (socklen_t)sizeof(socketAddress));
//                 }
//             }
//             close(socketDescriptor);
//         }
//         std::this_thread::sleep_for(std::chrono::seconds(5));
//     }
// }


int main(int argc, char **argv){
    rclcpp::init(argc,argv);

    nodeHandle = rclcpp::Node::make_shared("communication");
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting communication node");

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
    auto talon1Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>(
            "talon_14_info", 1,
            [](const messages::msg::TalonStatus::SharedPtr msg) {
                talonStatusCallback("Talon 1", msg);
            });

    auto talon2Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>(
            "talon_15_info", 1,
            [](const messages::msg::TalonStatus::SharedPtr msg) {
                talonStatusCallback("Talon 2", msg);
            });

    auto talon3Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>(
            "talon_16_info", 1,
            [](const messages::msg::TalonStatus::SharedPtr msg) {
                talonStatusCallback("Talon 3", msg);
            });

    auto talon4Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>(
            "talon_17_info", 1,
            [](const messages::msg::TalonStatus::SharedPtr msg) {
                talonStatusCallback("Talon 4", msg);
            });

    auto falcon1Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>(
            "talon_10_info", 1,
            [](const messages::msg::FalconStatus::SharedPtr msg) {
                falconStatusCallback("Falcon 1", msg);
            });

    auto falcon2Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>(
            "talon_11_info", 1,
            [](const messages::msg::FalconStatus::SharedPtr msg) {
                falconStatusCallback("Falcon 2", msg);
            });

    auto falcon3Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>(
            "talon_12_info", 1,
            [](const messages::msg::FalconStatus::SharedPtr msg) {
                falconStatusCallback("Falcon 3", msg);
            });

    auto falcon4Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>(
            "talon_13_info", 1,
            [](const messages::msg::FalconStatus::SharedPtr msg) {
                falconStatusCallback("Falcon 4", msg);
            });

    auto linearStatus1Subscriber = nodeHandle->create_subscription<messages::msg::LinearStatus>(
            "linearStatus1", 1,
            [](const messages::msg::LinearStatus::SharedPtr msg) {
                linearStatusCallback("Linear 1", msg);
            });

    auto linearStatus2Subscriber = nodeHandle->create_subscription<messages::msg::LinearStatus>(
            "linearStatus2", 1,
            [](const messages::msg::LinearStatus::SharedPtr msg) {
                linearStatusCallback("Linear 2", msg);
            });

    auto linearStatus3Subscriber = nodeHandle->create_subscription<messages::msg::LinearStatus>(
            "linearStatus3", 1,
            [](const messages::msg::LinearStatus::SharedPtr msg) {
                linearStatusCallback("Linear 3", msg);
            });

    auto linearStatus4Subscriber = nodeHandle->create_subscription<messages::msg::LinearStatus>(
            "linearStatus4", 1,
            [](const messages::msg::LinearStatus::SharedPtr msg) {
                linearStatusCallback("Linear 4", msg);
            });

    auto zedPositionSubscriber = nodeHandle->create_subscription<messages::msg::ZedPosition>("zed_position",1,zedPositionCallback);
    auto autonomyStatusSubscriber = nodeHandle->create_subscription<messages::msg::AutonomyStatus>("autonomy_status", 10, autonomyStatusCallback);
    auto systemStatusSubscriber = nodeHandle->create_subscription<messages::msg::SystemStatus>("system_status",10,systemStatusCallback);

    int server_fd, bytesRead; 
    int opt = 1; 
    uint8_t buffer[1024] = {0}; 
    std::string hello("Hello from server");

    // Creating socket file descriptor, handling errors
    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0) { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    }
    new_socket = server_fd; //This is the socket that will be used by the other functions above
    // std::thread broadcastThread(broadcastIP); hopefully don't need this anymore

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

    // broadcast=false;
    bytesRead = recvfrom(server_fd, buffer, 1024, 0, (struct sockaddr *)&address, &addrlen); 
    sendto(server_fd, hello.c_str(), strlen(hello.c_str()), 0, (struct sockaddr *)&address, addrlen); 
    silentRunning=true;

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
            }
        }
        if (isClientConnected) {
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = now - previousHeartbeat;

            if (elapsed.count() > 5.0) {
                isClientConnected = false;
                RCLCPP_INFO(nodeHandle->get_logger(), "Client disconnected");
                silentRunning = true;
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
            if(command==0){
                previousHeartbeat = std::chrono::high_resolution_clock::now();
                RCLCPP_INFO(nodeHandle->get_logger(), "Received Heartbeat");
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

    // broadcastThread.join(); hopefully don't need this anymore
}
