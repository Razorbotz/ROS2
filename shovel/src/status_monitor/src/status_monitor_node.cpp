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

std::shared_ptr<rclcpp::Publisher<messages::msg::SystemStatus_<std::allocator<void> >, std::allocator<void> > > systemStatusPublisher;


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

    check_packet_status("can0", "RX", previousRX, canMessage, buffer2);
    check_packet_status("can1", "RX", previousRX2, canMessage2, buffer2);
    check_packet_status("can0", "TX", previousTX, canMessage, buffer2);
    check_packet_status("can1", "TX", previousTX2, canMessage2, buffer2, true, usingCAN1);

    publishStatus();
}


// CAN ID is lower 6 bits
unsigned int parseID(struct can_frame frame){
    return frame.can_id & 0x0000002F;
}


std::mutex queue_mutex;
std::atomic<bool> run_threads{true};

void can_read_loop(const std::string &iface_name) {
    int s;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;

	const char *ifname = "CAN0";

	if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror(("Error opening socket on " + iface_name).c_str());
		return -1;
	}

	strcpy(ifr.ifr_name, ifname);

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror(("ioctl error on " + iface_name).c_str());
        close(s);
        return -1;
    }

	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror(("Bind error on " + iface_name).c_str());
        close(s);
        return -2;
	}

    int counter = 0;

    while (run_threads) {
        int nbytes = read(s, &frame, sizeof(frame));
        if (nbytes > 0) {
            std::lock_guard<std::mutex> lock(queue_mutex);
            if(counter % 20 == 0){
                RCLCPP_INFO(nodeHandle->get_logger(), "Received frame with id: 0x%X", frame.can_id);
            }
        }
    }
    close(s);
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
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting communication node");

    systemStatusPublisher = nodeHandle->create_publisher<messages::msg::SystemStatus>("system_status",1);

    getInterfaceName();

    std::thread can0_thread(can_read_loop, "can0");
    std::thread can1_thread(can_read_loop, "can1");

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
