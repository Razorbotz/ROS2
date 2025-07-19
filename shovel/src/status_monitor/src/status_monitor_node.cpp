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
#include <messages/msg/system_status.hpp>

rclcpp::Node::SharedPtr nodeHandle;

int rssi = 0;
std::string result = "";
char buffer2[128];
int previousTX = 0;
int previousRX = 0;
std::string canMessage = "";
char wifiCommand[128];
bool usingCAN1 = false;
int downCounter = 0;
std::string interfaceName = "wlan0";

std::shared_ptr<rclcpp::Publisher<messages::msg::SystemStatus_<std::allocator<void> >, std::allocator<void> > > systemStatusPublisher;


void publishStatus(){
    messages::msg::SystemStatus systemStatus;
    systemStatus.rssi = rssi;
    systemStatus.can_message = canMessage;
    systemStatus.rx_packets = previousRX;
    systemStatus.tx_packets = previousTX;
    systemStatus.using_can1 = usingCAN1;
}

void communicationInterval(){
    FILE* pipe = popen(wifiCommand, "r");
    result = "";
    while(!feof(pipe)){
        if(fgets(buffer2, 128, pipe) != nullptr){
            result += buffer2;
        }
    }
    rssi = ((int)result[2] - 48 ) * 10 + ((int)result[3] - 48);
    pclose(pipe);
    result = "";
    FILE* pipe2;
    if(!usingCAN1)
        pipe2 = popen("ip link show can0 | grep DOWN", "r");
    else
        pipe2 = popen("ip link show can1 | grep DOWN", "r");
    while(!feof(pipe2)){
        if(fgets(buffer2, 128, pipe2) != nullptr){
            result += buffer2;
        }
    }
    if(result.erase(result.find_last_not_of("\n\r") + 1).size() > 0){
        canMessage = "DOWN";
        int result;
        if(!usingCAN1)
            result = std::system("./../../restart_can.sh");
        else
            result = std::system("./../../restart_can1.sh");
        if (result == 0) {
            std::cout << "Restarted CAN interface" << std::endl;
            downCounter = 0;
        }
        else {
            std::cerr << "Failed to restart CAN interface" << std::endl;
            downCounter += 1;
            if(usingCAN1)
                RCLCPP_FATAL(nodeHandle->get_logger(), "CAN1 interface down after switch from CAN0.");
            if(downCounter == 5){
                if(!usingCAN1){
                    std::cout << "Switching from CAN0 interface to CAN1 interface" << std::endl;
                    int result = std::system("./../../launch_motors_can1.sh");
                    usingCAN1 = true;
                }
                else{
                    std::cout << "Switching from CAN0 interface to CAN1 interface" << std::endl;
                    int result = std::system("./../../launch_motors.sh");
                    usingCAN1 = false;
                }
                
            }
        }
    }
    else{
        canMessage = "UP";
        downCounter = 0;
    }
    result = "";
    pclose(pipe2);
    FILE* pipe3;
    if(!usingCAN1)
        pipe3 = popen("ifconfig can0 | grep -o -P '(?<=RX packets ).*(?= bytes)'", "r");
    else
        pipe3 = popen("ifconfig can1 | grep -o -P '(?<=RX packets ).*(?= bytes)'", "r");
    while(!feof(pipe3)){
        if(fgets(buffer2, 128, pipe3) != nullptr){
            result += buffer2;
        }
    }
    int rx = 0;
    for(int i = 0; i < result.size(); i++){
        if(result[i] == ' ')
            break;
        rx = rx * 10 + ((int) result[i] - 48);
    }
    if(previousRX == rx){
        canMessage = "RX ERROR";
    }
    previousRX = rx;
    result = "";
    pclose(pipe3);
    FILE* pipe4;
    if(!usingCAN1)
        pipe4 = popen("ifconfig can0 | grep -o -P '(?<=TX packets ).*(?= bytes)'", "r");
    else
        pipe4 = popen("ifconfig can1 | grep -o -P '(?<=TX packets ).*(?= bytes)'", "r");
    while(!feof(pipe4)){
        if(fgets(buffer2, 128, pipe4) != nullptr){
            result += buffer2;
        }
    }
    int tx = 0;
    for(int i = 0; i < result.size(); i++){
        if(result[i] == ' ')
            break;
        tx = tx * 10 + ((int) result[i] - 48);
    }
    if(previousTX == tx){
        canMessage = "TX ERROR";
    }
    previousTX = tx;
    result = "";
    pclose(pipe4);
    publishStatus();
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);

    nodeHandle = rclcpp::Node::make_shared("communication");
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting communication node");

    systemStatusPublisher = nodeHandle->create_publisher<messages::msg::SystemStatus>("system_status",1);

    FILE* pipe = popen("iw dev | awk '$1==\"Interface\"{print $2}'", "r");
    while(!feof(pipe)){
        if(fgets(buffer2, 128, pipe) != nullptr){
            result += buffer2;
        }
    }
    interfaceName = result.erase(result.find_last_not_of("\n\r") + 1);
    result = "";
    pclose(pipe);

    FILE* pipe2 = popen("grep 'VERSION_ID' /etc/os-release | cut -d '\"' -f 2", "r");
    while(!feof(pipe)){
        if(fgets(buffer2, 128, pipe) != nullptr){
            result += buffer2;
        }
    }
    if(result.erase(result.find_last_not_of("\n\r") + 1) == "20.04"){
        RCLCPP_INFO(nodeHandle->get_logger(), "Running Ubuntu 20.04");
        std::snprintf(wifiCommand, sizeof(wifiCommand), "iw dev %s link | grep -o -E ' -.{0,2}'", interfaceName.c_str());
    }
    else{
        RCLCPP_INFO(nodeHandle->get_logger(), "Running Ubuntu 22.04");
        std::snprintf(wifiCommand, sizeof(wifiCommand), "iwconfig %s | grep -E -o '=-.{0,2}'", interfaceName.c_str());
    }
    result = "";
    pclose(pipe2);

    rclcpp::Rate rate(10);
    while(rclcpp::ok()){

        rclcpp::spin_some(nodeHandle);
        communicationInterval();
        rate.sleep();
    }
}
