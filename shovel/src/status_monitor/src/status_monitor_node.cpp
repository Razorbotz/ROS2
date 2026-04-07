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
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <messages/msg/system_status.hpp>

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include "utils/utils.hpp"

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

// Maximum number of motors the system supports (message array size)
constexpr size_t MAX_MOTORS = 8;

// Actual number of motors in this configuration (5 or 6), set from parameter
size_t numMotors = 6;

// Motor CAN IDs in physical wiring order along the daisy chain.
// Index 0 = closest to CAN0 interface, last index = closest to CAN1 interface.
// Set from the "motor_wiring_order" parameter.
std::vector<uint32_t> motorWiringOrder;

// Per-motor status arrays (indexed by wiring position)
int motors0[MAX_MOTORS] = {0};
int motors1[MAX_MOTORS] = {0};
int copy0[MAX_MOTORS] = {0};
int copy1[MAX_MOTORS] = {0};
int interfaces[MAX_MOTORS] = {0}; // 0 = CAN0, 1 = CAN1 only, -1 = unreachable

// Lookup: CAN ID -> wiring index
std::unordered_map<uint32_t, size_t> canIdToWiringIndex;

std::shared_ptr<rclcpp::Publisher<messages::msg::SystemStatus_<std::allocator<void>>, std::allocator<void>>> systemStatusPublisher;
bool printData = false;
bool simulationMode = false;
std::string status = "";
int firstMotor = -1;
int secondMotor = -1;
int numBreaks = 0;

int numMotors0 = 0;
int numMotors1 = 0;

const uint32_t STATUS_01 = 0x041400;
const uint32_t STATUS_02 = 0x041440;
const uint32_t STATUS_03 = 0x041480;
const uint32_t STATUS_04 = 0x0414C0;

std::mutex mutex0, mutex1;
std::atomic<bool> run_threads{true};

void publishStatus() {
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

void publishSimulationStatus() {
    messages::msg::SystemStatus systemStatus;
    systemStatus.rssi = 0;
    systemStatus.can_message = "SIM";
    systemStatus.rx_packets = 0;
    systemStatus.tx_packets = 0;
    systemStatus.can2_message = "SIM";
    systemStatus.rx2_packets = 0;
    systemStatus.tx2_packets = 0;
    systemStatus.using_can1 = false;
    systemStatus.first_motor = -1;
    systemStatus.second_motor = -1;
    systemStatus.num_breaks = 0;
    // Mark all configured motors as visible on both interfaces
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        systemStatus.motors0[i] = (i < numMotors) ? 1 : 0;
        systemStatus.motors1[i] = (i < numMotors) ? 1 : 0;
        systemStatus.interfaces[i] = (i < numMotors) ? 0 : -1;
    }
    systemStatusPublisher->publish(systemStatus);
}

int extract_packet_count(const std::string& command, char* buffer) {
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) return -1;

    std::string pipeResult;
    while (!feof(pipe)) {
        if (fgets(buffer, 128, pipe) != nullptr) {
            pipeResult += buffer;
        }
    }
    pclose(pipe);

    int value = 0;
    for (size_t i = 0; i < pipeResult.size(); i++) {
        if (pipeResult[i] == ' ')
            break;
        value = value * 10 + ((int)pipeResult[i] - 48);
    }
    return value;
}

void check_packet_status(const std::string& interface, const std::string& direction,
                          int& previousValue, std::string& message, char* buffer,
                          bool onlyIfUsingCAN1 = false, bool can1Active = false) {
    int value = extract_packet_count(
        "ifconfig " + interface + " | grep -o -P '(?<=" + direction + " packets ).*(?= bytes)'",
        buffer);
    if ((!onlyIfUsingCAN1 || can1Active) && value == previousValue) {
        message = direction + " ERROR";
    }
    previousValue = value;
}

// CAN ID is lower 6 bits of the extended ID
unsigned int parseID(struct can_frame frame) {
    return frame.can_id & 0x0000003F;
}

bool get_motor_index(uint32_t can_id, size_t& index) {
    auto it = canIdToWiringIndex.find(can_id);
    if (it != canIdToWiringIndex.end()) {
        index = it->second;
        return true;
    }
    return false;
}

void can_read_loop(const std::string& iface_name, int (&motors)[MAX_MOTORS], std::mutex& mutex) {
    int s;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror(("Error opening socket on " + iface_name).c_str());
        return;
    }
    strcpy(ifr.ifr_name, iface_name.c_str());

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror(("ioctl error on " + iface_name).c_str());
        close(s);
        return;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    printf("%s at index %d\n", iface_name.c_str(), ifr.ifr_ifindex);

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror(("Bind error on " + iface_name).c_str());
        close(s);
        return;
    }

    while (run_threads) {
        int nbytes = read(s, &frame, sizeof(frame));
        if (nbytes > 0) {
            size_t motor_index;
            uint32_t statusField = frame.can_id & 0x000FFFC0;
            if (statusField == STATUS_01 || statusField == STATUS_02 || statusField == STATUS_04) {
                uint32_t canId = frame.can_id & 0x0000003F;
                if (get_motor_index(canId, motor_index)) {
                    std::lock_guard<std::mutex> lock(mutex);
                    motors[motor_index] = 1;
                }
            }
        }
    }
    close(s);
}


void checkInterfaceStatus() {
    // Snapshot and reset the per-motor visibility arrays
    {
        std::lock_guard<std::mutex> lock(mutex0);
        for (size_t i = 0; i < MAX_MOTORS; ++i) {
            copy0[i] = motors0[i];
            motors0[i] = 0;
        }
    }
    {
        std::lock_guard<std::mutex> lock(mutex1);
        for (size_t i = 0; i < MAX_MOTORS; ++i) {
            copy1[i] = motors1[i];
            motors1[i] = 0;
        }
    }

    // Count motors visible on each interface and classify per-motor reachability
    numMotors0 = 0;
    numMotors1 = 0;
    numBreaks = 0;
    firstMotor = -1;
    secondMotor = -1;

    for (size_t i = 0; i < numMotors; i++) {
        bool onCan0 = (copy0[i] == 1);
        bool onCan1 = (copy1[i] == 1);

        if (onCan0) numMotors0++;
        if (onCan1) numMotors1++;

        if (onCan0 && onCan1) {
            interfaces[i] = 0; // Reachable on both (nominal)
        } else if (onCan0 && !onCan1) {
            interfaces[i] = 0; // Only on CAN0
        } else if (!onCan0 && onCan1) {
            interfaces[i] = 1; // Only on CAN1
        } else {
            interfaces[i] = -1; // Unreachable on both
        }

        if (printData) {
            RCLCPP_INFO(nodeHandle->get_logger(),
                "Motor %d (0x%X): CAN0=%s, CAN1=%s, interface=%d",
                (int)i, motorWiringOrder[i],
                onCan0 ? "yes" : "no",
                onCan1 ? "yes" : "no",
                interfaces[i]);
        }
    }

    // Clear remaining slots
    for (size_t i = numMotors; i < MAX_MOTORS; i++) {
        interfaces[i] = -1;
    }

    if (numMotors0 == (int)numMotors && numMotors1 == (int)numMotors) {
        // All motors visible on both interfaces: no breaks
        status = "All motors reachable on both interfaces";
        numBreaks = 0;
        if (printData) {
            RCLCPP_INFO(nodeHandle->get_logger(), "%s", status.c_str());
        }
        return;
    }

    if (numMotors0 == 0 && numMotors1 == 0) {
        status = "Power failure - no motors detected on either interface";
        numBreaks = -1;
        if (printData) {
            RCLCPP_INFO(nodeHandle->get_logger(), "%s", status.c_str());
        }
        return;
    }

    if (numMotors0 == 0 && numMotors1 > 0) {
        status = "CAN0 interface sees no motors. Possible: cable disconnected from CAN0, "
                 "break between CAN0 connector and first motor (0x"
                 + std::to_string(motorWiringOrder[0]) + "), or CAN wires swapped";
        numBreaks = 1;
        firstMotor = -1;  // Break is before the first motor
        secondMotor = (int)motorWiringOrder[0];
        if (printData) {
            RCLCPP_INFO(nodeHandle->get_logger(), "%s", status.c_str());
        }
        return;
    }

    if (numMotors1 == 0 && numMotors0 > 0) {
        status = "CAN1 interface sees no motors. Possible: cable disconnected from CAN1, "
                 "break between last motor (0x"
                 + std::to_string(motorWiringOrder[numMotors - 1]) + ") and CAN1 connector";
        numBreaks = 1;
        firstMotor = (int)motorWiringOrder[numMotors - 1];
        secondMotor = -1;  // Break is after the last motor
        if (printData) {
            RCLCPP_INFO(nodeHandle->get_logger(), "%s", status.c_str());
        }
        return;
    }

    // Both interfaces see some motors. Walk the wiring order to find transitions.
    // In a single-break scenario:
    //   - CAN0 sees a contiguous block from the start of the chain
    //   - CAN1 sees a contiguous block from the end of the chain
    //   - Motors in between are unreachable (multiple breaks) or the sets are complementary (single break)

    // Find the last motor visible on CAN0 (scanning from start)
    int lastOnCan0 = -1;
    for (size_t i = 0; i < numMotors; i++) {
        if (copy0[i] == 1) {
            lastOnCan0 = (int)i;
        } else {
            break;  // First gap from CAN0 side
        }
    }

    // Find the first motor visible on CAN1 (scanning from end)
    int firstOnCan1 = (int)numMotors;
    for (int i = (int)numMotors - 1; i >= 0; i--) {
        if (copy1[i] == 1) {
            firstOnCan1 = i;
        } else {
            break;  // First gap from CAN1 side
        }
    }

    // Check for single break: CAN0 block [0..lastOnCan0] and CAN1 block [firstOnCan1..numMotors-1]
    // should be complementary and adjacent
    if (lastOnCan0 + 1 == firstOnCan1 && (size_t)(lastOnCan0 + 1 + ((int)numMotors - firstOnCan1)) == numMotors) {
        // Single break between lastOnCan0 and firstOnCan1
        numBreaks = 1;
        firstMotor = (int)motorWiringOrder[lastOnCan0];
        secondMotor = (int)motorWiringOrder[firstOnCan1];
        status = "Single break between motor 0x" + std::to_string(firstMotor)
               + " (position " + std::to_string(lastOnCan0) + ")"
               + " and motor 0x" + std::to_string(secondMotor)
               + " (position " + std::to_string(firstOnCan1) + ")";
        if (printData) {
            RCLCPP_INFO(nodeHandle->get_logger(), "%s", status.c_str());
        }
    } else {
        // Multiple breaks or non-contiguous visibility
        numBreaks = 0;

        // Count actual breaks by looking for transitions in combined visibility
        // Walk the chain and find gaps
        std::string breakLocations;
        bool prevSeen = true; // Assume connection at CAN0 end
        for (size_t i = 0; i < numMotors; i++) {
            bool currentSeen = (copy0[i] == 1 || copy1[i] == 1);
            if (prevSeen && !currentSeen) {
                numBreaks++;
                // Break is before this motor
                if (!breakLocations.empty()) breakLocations += "; ";
                if (i > 0) {
                    breakLocations += "between motor 0x" + std::to_string(motorWiringOrder[i - 1])
                                   + " and motor 0x" + std::to_string(motorWiringOrder[i]);
                } else {
                    breakLocations += "before first motor 0x" + std::to_string(motorWiringOrder[0]);
                }
            } else if (!prevSeen && currentSeen) {
                numBreaks++;
                // Break is before this motor (end of a dead zone)
                if (!breakLocations.empty()) breakLocations += "; ";
                breakLocations += "between motor 0x" + std::to_string(motorWiringOrder[i - 1])
                               + " and motor 0x" + std::to_string(motorWiringOrder[i]);
            }
            prevSeen = currentSeen;
        }

        // Check if there's a break after the last motor (CAN1 can't reach end)
        if (!prevSeen) {
            // Last motor(s) unreachable - already counted above
        }

        status = "Multiple breaks detected (" + std::to_string(numBreaks) + "): " + breakLocations;

        // Log motors only reachable on CAN1
        for (size_t i = 0; i < numMotors; i++) {
            if (copy0[i] == 0 && copy1[i] == 1) {
                RCLCPP_WARN(nodeHandle->get_logger(),
                    "Motor 0x%X (position %d) only reachable on CAN1",
                    motorWiringOrder[i], (int)i);
            }
            if (copy0[i] == 0 && copy1[i] == 0) {
                RCLCPP_ERROR(nodeHandle->get_logger(),
                    "Motor 0x%X (position %d) unreachable on both interfaces!",
                    motorWiringOrder[i], (int)i);
            }
        }

        if (printData) {
            RCLCPP_INFO(nodeHandle->get_logger(), "%s", status.c_str());
        }
    }

    // Log CAN1-only motors for all cases
    for (size_t i = 0; i < numMotors; i++) {
        if (interfaces[i] == 1 && printData) {
            RCLCPP_INFO(nodeHandle->get_logger(),
                "Motor 0x%X (position %d) is only readable on CAN1",
                motorWiringOrder[i], (int)i);
        }
    }
}

void getInterfaceName() {
    FILE* pipe = popen("iw dev | awk '$1==\"Interface\"{print $2}'", "r");
    result = "";
    while (!feof(pipe)) {
        if (fgets(buffer2, 128, pipe) != nullptr) {
            result += buffer2;
        }
    }
    interfaceName = result.erase(result.find_last_not_of("\n\r") + 1);
    result = "";
    pclose(pipe);

    FILE* pipe2 = popen("grep 'VERSION_ID' /etc/os-release | cut -d '\"' -f 2", "r");
    result = "";
    while (!feof(pipe2)) {
        if (fgets(buffer2, 128, pipe2) != nullptr) {
            result += buffer2;
        }
    }
    if (result.erase(result.find_last_not_of("\n\r") + 1) == "20.04") {
        RCLCPP_INFO(nodeHandle->get_logger(), "Running Ubuntu 20.04");
        std::snprintf(wifiCommand, sizeof(wifiCommand),
            "iw dev %s link | grep -o -E ' -.{0,2}'", interfaceName.c_str());
    } else {
        RCLCPP_INFO(nodeHandle->get_logger(), "Running Ubuntu 22.04");
        std::snprintf(wifiCommand, sizeof(wifiCommand),
            "iwconfig %s | grep -E -o '=-.{0,2}'", interfaceName.c_str());
    }
    result = "";
    pclose(pipe2);
}

void statusCheck() {
    FILE* pipe = popen(wifiCommand, "r");
    result = "";
    while (!feof(pipe)) {
        if (fgets(buffer2, 128, pipe) != nullptr) {
            result += buffer2;
        }
    }
    if (result.size() >= 4) {
        rssi = ((int)result[2] - 48) * 10 + ((int)result[3] - 48);
    }
    pclose(pipe);
    result = "";

    check_packet_status("can0", "RX", previousRX, canMessage, buffer2);
    check_packet_status("can1", "RX", previousRX2, canMessage2, buffer2);
    check_packet_status("can0", "TX", previousTX, canMessage, buffer2);
    check_packet_status("can1", "TX", previousTX2, canMessage2, buffer2, true, usingCAN1);

    checkInterfaceStatus();

    publishStatus();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    nodeHandle = rclcpp::Node::make_shared("status_monitor");
    RCLCPP_INFO(nodeHandle->get_logger(), "Starting status monitor node");

    systemStatusPublisher = nodeHandle->create_publisher<messages::msg::SystemStatus>("system_status", 1);
    printData = utils::getParameter<bool>(nodeHandle, "print_data", false);
    simulationMode = utils::getParameter<bool>(nodeHandle, "simulation", false);

    // Number of motors in this robot configuration (5 or 6)
    int numMotorsParam = utils::getParameter<int>(nodeHandle, "num_motors", 6);
    if (numMotorsParam < 1 || numMotorsParam > (int)MAX_MOTORS) {
        RCLCPP_FATAL(nodeHandle->get_logger(),
            "num_motors must be between 1 and %d, got %d", (int)MAX_MOTORS, numMotorsParam);
        return 1;
    }
    numMotors = (size_t)numMotorsParam;

    // In simulation mode, skip all CAN/WiFi setup and just publish healthy status
    if (simulationMode) {
        RCLCPP_INFO(nodeHandle->get_logger(),
            "Running in SIMULATION mode — publishing all-healthy status for %zu motors", numMotors);

        rclcpp::Rate rate(20);
        while (rclcpp::ok()) {
            rclcpp::spin_some(nodeHandle);
            publishSimulationStatus();
            rate.sleep();
        }
        rclcpp::shutdown();
        return 0;
    }

    // Motor CAN IDs in physical wiring order (closest to CAN0 first).
    // Pass as an integer array parameter, e.g.: [10, 11, 13, 12, 16, 14]
    // These correspond to hex CAN IDs: 0xA, 0xB, 0xD, 0xC, 0x10, 0xE
    // Motor CAN IDs in physical wiring order (closest to CAN0 first).
    // Pass as an integer array parameter, e.g.: [10, 11, 13, 12, 16, 14]
    // These correspond to hex CAN IDs: 0xA, 0xB, 0xD, 0xC, 0x10, 0xE
    std::vector<int64_t> defaultWiring = {0xA, 0xB, 0xD, 0xC, 0x10, 0xE};
    std::vector<int64_t> wiringParam = utils::getParameter<std::vector<int64_t>>(
        nodeHandle, "motor_wiring_order", defaultWiring);
    
    if (wiringParam.size() != numMotors) {
        RCLCPP_FATAL(nodeHandle->get_logger(),
            "motor_wiring_order has %zu entries but num_motors is %zu",
            wiringParam.size(), numMotors);
        return 1;
    }

    motorWiringOrder.resize(numMotors);
    for (size_t i = 0; i < numMotors; i++) {
        motorWiringOrder[i] = (uint32_t)wiringParam[i];
    }

    // Build lookup map: CAN ID -> wiring position index
    canIdToWiringIndex.clear();
    for (size_t i = 0; i < numMotors; ++i) {
        canIdToWiringIndex[motorWiringOrder[i]] = i;
        RCLCPP_INFO(nodeHandle->get_logger(),
            "Wiring position %zu: motor CAN ID 0x%X", i, motorWiringOrder[i]);
    }

    // Zero out motor arrays
    for (size_t i = 0; i < MAX_MOTORS; i++) {
        motors0[i] = 0;
        motors1[i] = 0;
        copy0[i] = 0;
        copy1[i] = 0;
        interfaces[i] = -1;
    }

    getInterfaceName();

    std::thread can0_thread(can_read_loop, "can0", std::ref(motors0), std::ref(mutex0));
    std::thread can1_thread(can_read_loop, "can1", std::ref(motors1), std::ref(mutex1));

    rclcpp::Rate rate(20);
    while (rclcpp::ok()) {
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