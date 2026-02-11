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

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <chrono>
#include "utils/utils.hpp"

// TODO: Check if the interface is up
// If down, restart interface
// Check behavior when the wires are flipped
// Check behavior in total power loss, individual power loss
// Check behavior with multiple breaks in lines

// NOTE: Need to investigate whether the system will randomly decide which interface is CAN0 vs CAN1. The paths are 
// hardcoded around knowing which interface is where. Will look into location agnostic code or setting interface by ID
rclcpp::Node::SharedPtr nodeHandle;

int rssi = 0;
std::string result = "";
char buffer2[128];
int previousTX = 0;
int previousRX = 0;
int previousRX2 = 0;
int previousTX2 = 0;
std::string canMessage = "";
std::string canMessage2 = "";
char wifiCommand[128];
bool usingCAN1 = false;
int downCounter = 0;
std::string interfaceName = "wlan0";

constexpr size_t NUM_MOTORS = 8;

int motors0[NUM_MOTORS] = {1, 1, 1, 1, 0, 0, 0, 0};
int motors1[NUM_MOTORS] = {0, 0, 0, 0, 1, 1, 1, 1};
int copy0[NUM_MOTORS] = {0};
int copy1[NUM_MOTORS] = {0};
int interfaces[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0};

const std::array<uint32_t, NUM_MOTORS> MOTOR_IDS = {0xA, 0XB, 0xD, 0xC, 0x10, 0xE};

std::shared_ptr<rclcpp::Publisher<messages::msg::SystemStatus_<std::allocator<void> >, std::allocator<void> > > systemStatusPublisher;
bool printData = false;
std::string status = "";
int firstMotor = -1;
int secondMotor = -1;
int numBreaks = 0;

bool switched = false;
int numMotors0 = 0;
int numMotors1 = 0;

const uint32_t STATUS_01 = 0x041400;
const uint32_t STATUS_02 = 0x041440;
const uint32_t STATUS_03 = 0x041480;
const uint32_t STATUS_04 = 0x0414C0;


void publishStatus(){
    messages::msg::SystemStatus systemStatus;
    systemStatus.rssi = rssi;
    systemStatus.can_message = canMessage;
    systemStatus.rx_packets = previousRX;
    systemStatus.tx_packets = previousTX;
    systemStatus.can2_message = canMessage2;
    systemStatus.rx2_packets = previousRX2;
    systemStatus.tx2_packets = previousTX2;
    systemStatus.using_can1 = usingCAN1;
    systemStatus.first_motor = firstMotor;
    systemStatus.second_motor = secondMotor;
    systemStatus.num_breaks = numBreaks;
    std::copy(std::begin(motors0), std::end(motors0), systemStatus.motors0.begin());
    std::copy(std::begin(motors1), std::end(motors1), systemStatus.motors1.begin());
    std::copy(std::begin(interfaces), std::end(interfaces), systemStatus.interfaces.begin());
    systemStatusPublisher->publish(systemStatus);
}

int main(int argc, char **argv){
    rclcpp::init(argc,argv);

    nodeHandle = rclcpp::Node::make_shared("status_monitor");
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting status monitor node");

    systemStatusPublisher = nodeHandle->create_publisher<messages::msg::SystemStatus>("system_status",1);
    printData = utils::getParameter<bool>(nodeHandle, "print_data", false);

    rclcpp::Rate rate(10);
    while(rclcpp::ok()){
        publishStatus();
        rclcpp::spin_some(nodeHandle);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
