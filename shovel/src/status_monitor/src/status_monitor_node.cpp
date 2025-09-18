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

constexpr size_t NUM_MOTORS = 6;

int motors0[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
int motors1[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
int copy0[NUM_MOTORS] = {0};
int copy1[NUM_MOTORS] = {0};
int interfaces[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};

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
    systemStatusPublisher->publish(systemStatus);
}

int extract_packet_count(const std::string& command, char* buffer2) {
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) return -1;

    std::string result;
    while (!feof(pipe)) {
        if (fgets(buffer2, 128, pipe) != nullptr) {
            result += buffer2;
        }
    }
    pclose(pipe);

    int value = 0;
    for(int i = 0; i < result.size(); i++){
        if(result[i] == ' ')
            break;
        value = value * 10 + ((int) result[i] - 48);
    }
    return value;
}

void check_packet_status(const std::string& interface, const std::string& direction, int& previousValue, std::string& message, char* buffer, bool onlyIfUsingCAN1 = false, bool usingCAN1 = false) {
    int value = extract_packet_count("ifconfig " + interface + " | grep -o -P '(?<=" + direction + " packets ).*(?= bytes)'", buffer);
    if ((!onlyIfUsingCAN1 || usingCAN1) && value == previousValue) {
        message = direction + " ERROR";
    }
    previousValue = value;
}


// CAN ID is lower 6 bits
unsigned int parseID(struct can_frame frame){
    return frame.can_id & 0x0000003F;
}

// Lookup map: CAN ID to motor index
std::unordered_map<uint32_t, size_t> motor_id_to_index;

// Convert CAN ID to motor index
bool get_motor_index(uint32_t can_id, size_t &index) {
    auto it = motor_id_to_index.find(can_id);
    if (it != motor_id_to_index.end()) {
        index = it->second;
        return true;
    }
    return false;
}

std::mutex mutex0, mutex1;
std::atomic<bool> run_threads{true};

void can_read_loop(const std::string &iface_name, int (&motors)[NUM_MOTORS], std::mutex &mutex) {
    int s;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;

	if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror(("Error opening socket on " + iface_name).c_str());
		return;
	}
	strcpy(ifr.ifr_name, iface_name.c_str());

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror(("ioctl error on " + iface_name).c_str());
        close(s);
        return;
    }

	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", iface_name.c_str(), ifr.ifr_ifindex);

	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror(("Bind error on " + iface_name).c_str());
        close(s);
        return;
	}

    int counter = 0;

    while (run_threads) {
        int nbytes = read(s, &frame, sizeof(frame));
        if (nbytes > 0) {
            size_t motor_index;
            uint32_t status = frame.can_id & 0x000FFFC0;
            if(status == STATUS_01 || status == STATUS_02 || status == STATUS_04){
                if (get_motor_index(frame.can_id & 0x0000003F, motor_index)) {
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        motors[motor_index] = 1;
                    }
                }
            }
            
        }
    }
    close(s);
}

void switchInterfaces(){
    for(int i = NUM_MOTORS; i >= numMotors0; i--){
        if(interfaces[i] == 0){
            interfaces[i] = 1;
            RCLCPP_INFO(nodeHandle->get_logger(), "Killing motor %d", MOTOR_IDS[i]);
            std::string killCommand = "";
            std::string startCommand = "./launch/launch_node/can1/launch_";
            if(i > 3){
                killCommand = "pkill -f Talon" + std::to_string(MOTOR_IDS[i]);
                startCommand += "talon_" + std::to_string(MOTOR_IDS[i]) + ".sh";
            }
            else{
                killCommand = "pkill -f Falcon" + std::to_string(MOTOR_IDS[i]);
                startCommand += "falcon_" + std::to_string(MOTOR_IDS[i]) + ".sh";
            }
            RCLCPP_INFO(nodeHandle->get_logger(), "killCommand: %s", killCommand.c_str());
            RCLCPP_INFO(nodeHandle->get_logger(), "startCommand: %s", startCommand.c_str());
            int result = 0;
            result = std::system(killCommand.c_str());
            if (result == 0) {
                RCLCPP_INFO(nodeHandle->get_logger(), "Killed process with kill command");
            }
            std::thread([startCommand]() {
                std::system(startCommand.c_str());
            }).detach();
            RCLCPP_INFO(nodeHandle->get_logger(), "Started process with start command");
        }
    }
}


void checkInterfaceStatus(){
    {
        std::lock_guard<std::mutex> lock(mutex0);
        for (int i = 0; i < NUM_MOTORS; ++i) {
            copy0[i] = motors0[i];
            motors0[i] = 0;
        }
    }
    {
        std::lock_guard<std::mutex> lock(mutex1);
        for (int i = 0; i < NUM_MOTORS; ++i) {
            copy1[i] = motors1[i];
            motors1[i] = 0;
        }
    }
    numMotors0 = 0;
    numMotors1 = 0;
    for(int i = 0; i < NUM_MOTORS; i++){
        if(copy0[i] == 1){
            if(printData){
                RCLCPP_INFO(nodeHandle->get_logger(), "CAN 0: Motor %d heard.", i);
            }
            numMotors0 += 1;
        }
        else{
            if(printData){
                RCLCPP_INFO(nodeHandle->get_logger(), "CAN 0: Motor %d not heard.", i);
            }
        }
    }
    for(int i = 0; i < NUM_MOTORS; i++){
        if(copy1[i] == 1){
            if(printData){
                RCLCPP_INFO(nodeHandle->get_logger(), "CAN 1: Motor %d heard.", i);
            }
            numMotors1 += 1;
        }
        else{
            if(printData){
                RCLCPP_INFO(nodeHandle->get_logger(), "CAN 1: Motor %d not heard.", i);
            }
        }
    }

    if(numMotors0 == NUM_MOTORS && numMotors1 == NUM_MOTORS){
        if(printData){
            RCLCPP_INFO(nodeHandle->get_logger(), "CAN0 and CAN1 reading all motors correctly");
        }
        status = "CAN0 and CAN1 reading all motors correctly";
    }
    else{
        if(numMotors0 == NUM_MOTORS){
            if(printData){
                RCLCPP_INFO(nodeHandle->get_logger(), "CAN 0 reading all motors correctly");
            }
        }
        if(numMotors1 == NUM_MOTORS){
            if(printData){
                RCLCPP_INFO(nodeHandle->get_logger(), "CAN 1 reading all motors correctly");                
            }
        }
        if(numMotors0 == 0){
            if(numMotors1 == 0){
                if(printData){
                    RCLCPP_INFO(nodeHandle->get_logger(), "Power failure");
                }
                status = "Power failure";
            }
            else{
                //switchInterfaces();
                if(printData){{
                    RCLCPP_INFO(nodeHandle->get_logger(), "CAN wires pulled out of CAN0 interface, break in line just outside"
                    "of box, or CAN wires have been swapped before first motor");
                }
                    status = "CAN wires pulled out of CAN0 interface, break in line just outside"
                    "of box, or CAN wires have been swapped before first motor";
                }
                return;
            }
        }
        if(numMotors0 + numMotors1 == NUM_MOTORS){
            //switchInterfaces();
            if(printData){
                RCLCPP_INFO(nodeHandle->get_logger(), "Single break in line");
                RCLCPP_INFO(nodeHandle->get_logger(), "numMotors0: %d, numMotors1: %d", numMotors0, numMotors1);
            }
            // Identify where the break is
            if(numMotors0 > 0){
                if(printData){
                    RCLCPP_INFO(nodeHandle->get_logger(), "Break between motors %d and %d", MOTOR_IDS[numMotors0-1], MOTOR_IDS[numMotors0]);
                }
                status = "Break between motors %d and %d", MOTOR_IDS[numMotors0-1], MOTOR_IDS[numMotors0];
            }
        }
        else if(numMotors0 + numMotors1 < NUM_MOTORS){
            if(printData){
                RCLCPP_INFO(nodeHandle->get_logger(), "Multiple breaks in line");
                RCLCPP_INFO(nodeHandle->get_logger(), "Breaks between motors %d and %d", MOTOR_IDS[numMotors0], MOTOR_IDS[NUM_MOTORS-numMotors1]);
            }
            status = "Multiple breaks in line";
        }
        else{
            if(printData){
                RCLCPP_INFO(nodeHandle->get_logger(), "Odd things are happening. NumMotors0: %d, NumMotors1: %d", numMotors0, numMotors1);
            }
        }
    }
}


void statusCheck(){
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
    
    // This was originally intended to catch if the interface crashed, then
    // switch to CAN1. Because we're using CAN1 already, this isn't needed
    /*
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
    */

    check_packet_status("can0", "RX", previousRX, canMessage, buffer2);
    check_packet_status("can1", "RX", previousRX2, canMessage2, buffer2);
    check_packet_status("can0", "TX", previousTX, canMessage, buffer2);
    check_packet_status("can1", "TX", previousTX2, canMessage2, buffer2, true, usingCAN1);

    checkInterfaceStatus();

    publishStatus();
}


void getInterfaceName(){
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
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);

    nodeHandle = rclcpp::Node::make_shared("status_monitor");
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting status monitor node");

    systemStatusPublisher = nodeHandle->create_publisher<messages::msg::SystemStatus>("system_status",1);
    printData = utils::getParameter<bool>(nodeHandle, "print_data", false);

    getInterfaceName();

    for (size_t i = 0; i < MOTOR_IDS.size(); ++i){
        motor_id_to_index[MOTOR_IDS[i]] = i;
    }

    std::thread can0_thread(can_read_loop, "can0", std::ref(motors0), std::ref(mutex0));
    std::thread can1_thread(can_read_loop, "can1", std::ref(motors1), std::ref(mutex1));

    rclcpp::Rate rate(10);
    while(rclcpp::ok()){

        rclcpp::spin_some(nodeHandle);
        statusCheck();
        rate.sleep();
    }

    run_threads = false;
    if (can0_thread.joinable()) can0_thread.join();
    if (can1_thread.joinable()) can1_thread.join();

    rclcpp::shutdown();
    return 0;
}
