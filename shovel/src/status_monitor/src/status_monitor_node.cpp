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
int previousRX1 = 0;
int previousTX1 = 0;
int previousRX2 = 0;
int previousTX2 = 0;
std::string canMessage = "";
std::string canMessage1 = "";
std::string canMessage2 = "";
char wifiCommand[128];
bool usingCAN2 = false;
int downCounter = 0;
std::string interfaceName = "wlan0";

// Maximum number of motors the system supports (message array size)
constexpr size_t MAX_MOTORS = 8;

// Actual number of motors in this configuration (5 or 6), set from parameter
size_t numMotors = 6;

// Motor CAN IDs in physical wiring order along the daisy chain.
// Index 0 = closest to CAN1 interface, last index = closest to CAN2 interface.
// Set from the "motor_wiring_order" parameter.
std::vector<uint32_t> motorWiringOrder;

// Per-motor status arrays (indexed by wiring position)
// motors1 = visibility on CAN1 (physical, one side of bus)
// motors2 = visibility on CAN2 (physical, other side of bus)
int motors1[MAX_MOTORS] = {0};
int motors2[MAX_MOTORS] = {0};
int copy1[MAX_MOTORS] = {0};
int copy2[MAX_MOTORS] = {0};
int interfaces[MAX_MOTORS] = {0}; // 0 = CAN1 (or both), 1 = CAN2 only, -1 = unreachable

// Lookup: CAN ID -> wiring index
std::unordered_map<uint32_t, size_t> canIdToWiringIndex;

std::shared_ptr<rclcpp::Publisher<messages::msg::SystemStatus_<std::allocator<void>>, std::allocator<void>>> systemStatusPublisher;
bool printData = false;
bool simulationMode = false;
std::string status = "";
int firstMotor = -1;
int secondMotor = -1;
int numBreaks = 0;

int numMotors1 = 0;
int numMotors2 = 0;

const uint32_t STATUS_01 = 0x041400;
const uint32_t STATUS_02 = 0x041440;
const uint32_t STATUS_03 = 0x041480;
const uint32_t STATUS_04 = 0x0414C0;

std::mutex mutex1, mutex2;
std::atomic<bool> run_threads{true};

// CAN interface names — set from parameters
std::string vcanInterface = "can0";   // Virtual CAN (cangw routing)
std::string phys1Interface = "can1";  // Physical side 1 of bus
std::string phys2Interface = "can2";  // Physical side 2 of bus

// Lowest motor CAN ID in the system — used to convert a motor's CAN ID into
// its index in the published SystemStatus arrays (index = can_id - MOTOR_ID_BASE).
// Drive/arm motors use CAN IDs 10..17 (0xA..0x11), so the base is 10.
constexpr uint32_t MOTOR_ID_BASE = 10;

void publishStatus() {
    messages::msg::SystemStatus systemStatus;
    systemStatus.rssi = rssi;
    systemStatus.can_message = canMessage;       // CAN0 (vcan) status
    systemStatus.rx_packets = previousRX;
    systemStatus.tx_packets = previousTX;
    systemStatus.can2_message = canMessage1;      // CAN1 (physical) status
    systemStatus.rx2_packets = previousRX1;
    systemStatus.tx2_packets = previousTX1;
    systemStatus.using_can1 = usingCAN2;
    systemStatus.first_motor = firstMotor;
    systemStatus.second_motor = secondMotor;
    systemStatus.num_breaks = numBreaks;

    // Internally motors1/motors2/interfaces/copy1/copy2 are indexed by physical
    // WIRING POSITION along the daisy chain. Downstream consumers (communication
    // node / Aegis) index by MOTOR-ID OFFSET (can_id - MOTOR_ID_BASE). Remap here
    // so motor 0xN always lands in array slot (N - MOTOR_ID_BASE), regardless of
    // where it sits in the physical wiring order.
    //
    // We read from the already-snapshotted copy1/copy2 (populated by
    // checkInterfaceStatus, which runs immediately before publishStatus) rather
    // than from the live TTL arrays, so the published visibility is consistent
    // with the break-detection logic.
    uint8_t motors0_by_id[MAX_MOTORS] = {0};
    uint8_t motors1_by_id[MAX_MOTORS] = {0};
    int     interfaces_by_id[MAX_MOTORS];
    for (size_t i = 0; i < MAX_MOTORS; i++) interfaces_by_id[i] = -1;

    for (size_t i = 0; i < numMotors; i++) {
        uint32_t can_id = motorWiringOrder[i];
        if (can_id < MOTOR_ID_BASE) {
            RCLCPP_WARN_THROTTLE(nodeHandle->get_logger(), *nodeHandle->get_clock(), 5000,
                "Motor CAN ID 0x%X is below MOTOR_ID_BASE (%u); skipping in published status",
                can_id, MOTOR_ID_BASE);
            continue;
        }
        size_t idx = (size_t)(can_id - MOTOR_ID_BASE);
        if (idx >= MAX_MOTORS) {
            RCLCPP_WARN_THROTTLE(nodeHandle->get_logger(), *nodeHandle->get_clock(), 5000,
                "Motor CAN ID 0x%X maps to index %zu which exceeds MAX_MOTORS (%zu); skipping",
                can_id, idx, MAX_MOTORS);
            continue;
        }
        motors0_by_id[idx]    = (uint8_t)copy1[i];
        motors1_by_id[idx]    = (uint8_t)copy2[i];
        interfaces_by_id[idx] = interfaces[i];
    }

    std::copy(std::begin(motors0_by_id),    std::end(motors0_by_id),    systemStatus.motors0.begin());
    std::copy(std::begin(motors1_by_id),    std::end(motors1_by_id),    systemStatus.motors1.begin());
    std::copy(std::begin(interfaces_by_id), std::end(interfaces_by_id), systemStatus.interfaces.begin());
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
                          bool onlyIfActive = false, bool isActive = true) {
    int value = extract_packet_count(
        "ifconfig " + interface + " | grep -o -P '(?<=" + direction + " packets ).*(?= bytes)'",
        buffer);
    if ((!onlyIfActive || isActive) && value == previousValue) {
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
            uint32_t canId = frame.can_id & 0x0000003F;
            if (get_motor_index(canId, motor_index)) {
                std::lock_guard<std::mutex> lock(mutex);
                motors[motor_index] = 3;
            }
        }
    }
    close(s);
}


void checkInterfaceStatus() {
    // Snapshot and reset the per-motor visibility arrays
    {
        std::lock_guard<std::mutex> lock(mutex1);
        for (size_t i = 0; i < MAX_MOTORS; ++i) {
            copy1[i] = (motors1[i] > 0) ? 1 : 0;
            if (motors1[i] > 0) motors1[i]--; // Decay the TTL
        }
    }
    {
        std::lock_guard<std::mutex> lock(mutex2);
        for (size_t i = 0; i < MAX_MOTORS; ++i) {
            copy2[i] = (motors2[i] > 0) ? 1 : 0;
            if (motors2[i] > 0) motors2[i]--; // Decay the TTL
        }
    }

    // Count motors visible on each interface and classify per-motor reachability
    numMotors1 = 0;
    numMotors2 = 0;
    numBreaks = 0;
    firstMotor = -1;
    secondMotor = -1;

    for (size_t i = 0; i < numMotors; i++) {
        bool onCan1 = (copy1[i] == 1);
        bool onCan2 = (copy2[i] == 1);

        if (onCan1) numMotors1++;
        if (onCan2) numMotors2++;

        if (onCan1 && onCan2) {
            interfaces[i] = 0; // Reachable on both (nominal)
        } else if (onCan1 && !onCan2) {
            interfaces[i] = 0; // Only on CAN1
        } else if (!onCan1 && onCan2) {
            interfaces[i] = 1; // Only on CAN2
        } else {
            interfaces[i] = -1; // Unreachable on both
        }

        if (printData) {
            RCLCPP_INFO(nodeHandle->get_logger(),
                "Motor %d (0x%X): %s=%s, %s=%s, interface=%d",
                (int)i, motorWiringOrder[i],
                phys1Interface.c_str(), onCan1 ? "yes" : "no",
                phys2Interface.c_str(), onCan2 ? "yes" : "no",
                interfaces[i]);
        }
    }

    // Clear remaining slots
    for (size_t i = numMotors; i < MAX_MOTORS; i++) {
        interfaces[i] = -1;
    }

    if (numMotors1 == (int)numMotors && numMotors2 == (int)numMotors) {
        // All motors visible on both interfaces: no breaks
        status = "All motors reachable on both interfaces";
        numBreaks = 0;
        if (printData) {
            RCLCPP_INFO(nodeHandle->get_logger(), "%s", status.c_str());
        }
        return;
    }

    if (numMotors1 == 0 && numMotors2 == 0) {
        status = "Power failure - no motors detected on either interface";
        numBreaks = -1;
        if (printData) {
            RCLCPP_INFO(nodeHandle->get_logger(), "%s", status.c_str());
        }
        return;
    }

    if (numMotors1 == 0 && numMotors2 > 0) {
        status = phys1Interface + " sees no motors. Possible: cable disconnected from "
                 + phys1Interface + ", break between " + phys1Interface + " connector and first motor (0x"
                 + std::to_string(motorWiringOrder[0]) + "), or CAN wires swapped";
        numBreaks = 1;
        firstMotor = -1;  // Break is before the first motor
        secondMotor = (int)motorWiringOrder[0];
        if (printData) {
            RCLCPP_INFO(nodeHandle->get_logger(), "%s", status.c_str());
        }
        return;
    }

    if (numMotors2 == 0 && numMotors1 > 0) {
        status = phys2Interface + " sees no motors. Possible: cable disconnected from "
                 + phys2Interface + ", break between last motor (0x"
                 + std::to_string(motorWiringOrder[numMotors - 1]) + ") and "
                 + phys2Interface + " connector";
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
    //   - CAN1 sees a contiguous block from the start of the chain
    //   - CAN2 sees a contiguous block from the end of the chain
    //   - Motors in between are unreachable (multiple breaks) or the sets are complementary (single break)

    // Find the last motor visible on CAN1 (scanning from start)
    int lastOnCan1 = -1;
    for (size_t i = 0; i < numMotors; i++) {
        if (copy1[i] == 1) {
            lastOnCan1 = (int)i;
        } else {
            break;  // First gap from CAN1 side
        }
    }

    // Find the first motor visible on CAN2 (scanning from end)
    int firstOnCan2 = (int)numMotors;
    for (int i = (int)numMotors - 1; i >= 0; i--) {
        if (copy2[i] == 1) {
            firstOnCan2 = i;
        } else {
            break;  // First gap from CAN2 side
        }
    }

    // Check for single break: CAN1 block [0..lastOnCan1] and CAN2 block [firstOnCan2..numMotors-1]
    // should be complementary and adjacent
    if (lastOnCan1 + 1 == firstOnCan2 && (size_t)(lastOnCan1 + 1 + ((int)numMotors - firstOnCan2)) == numMotors) {
        // Single break between lastOnCan1 and firstOnCan2
        numBreaks = 1;
        firstMotor = (int)motorWiringOrder[lastOnCan1];
        secondMotor = (int)motorWiringOrder[firstOnCan2];
        status = "Single break between motor 0x" + std::to_string(firstMotor)
               + " (position " + std::to_string(lastOnCan1) + ")"
               + " and motor 0x" + std::to_string(secondMotor)
               + " (position " + std::to_string(firstOnCan2) + ")";
        if (printData) {
            RCLCPP_INFO(nodeHandle->get_logger(), "%s", status.c_str());
        }
    } else {
        // Multiple breaks or non-contiguous visibility
        numBreaks = 0;

        // Count actual breaks by looking for transitions in combined visibility
        // Walk the chain and find gaps
        std::string breakLocations;
        bool prevSeen = true; // Assume connection at CAN1 end
        for (size_t i = 0; i < numMotors; i++) {
            bool currentSeen = (copy1[i] == 1 || copy2[i] == 1);
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

        // Check if there's a break after the last motor (CAN2 can't reach end)
        if (!prevSeen) {
            // Last motor(s) unreachable - already counted above
        }

        status = "Multiple breaks detected (" + std::to_string(numBreaks) + "): " + breakLocations;

        // Log motors only reachable on CAN2
        for (size_t i = 0; i < numMotors; i++) {
            if (copy1[i] == 0 && copy2[i] == 1) {
                RCLCPP_WARN(nodeHandle->get_logger(),
                    "Motor 0x%X (position %d) only reachable on %s",
                    motorWiringOrder[i], (int)i, phys2Interface.c_str());
            }
            if (copy1[i] == 0 && copy2[i] == 0) {
                RCLCPP_ERROR(nodeHandle->get_logger(),
                    "Motor 0x%X (position %d) unreachable on both interfaces!",
                    motorWiringOrder[i], (int)i);
            }
        }

        if (printData) {
            RCLCPP_INFO(nodeHandle->get_logger(), "%s", status.c_str());
        }
    }

    // Log CAN2-only motors for all cases
    for (size_t i = 0; i < numMotors; i++) {
        if (interfaces[i] == 1 && printData) {
            RCLCPP_INFO(nodeHandle->get_logger(),
                "Motor 0x%X (position %d) is only readable on %s",
                motorWiringOrder[i], (int)i, phys2Interface.c_str());
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

    // CAN0 (vcan / cangw) packet stats
    check_packet_status(vcanInterface, "RX", previousRX, canMessage, buffer2);
    check_packet_status(vcanInterface, "TX", previousTX, canMessage, buffer2);

    // CAN1 (physical side 1) packet stats
    check_packet_status(phys1Interface, "RX", previousRX1, canMessage1, buffer2);
    check_packet_status(phys1Interface, "TX", previousTX1, canMessage1, buffer2);

    // CAN2 (physical side 2) packet stats
    check_packet_status(phys2Interface, "RX", previousRX2, canMessage2, buffer2, true, usingCAN2);
    check_packet_status(phys2Interface, "TX", previousTX2, canMessage2, buffer2, true, usingCAN2);

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

    vcanInterface = utils::getParameter<std::string>(nodeHandle, "vcan_interface", "can0");
    phys1Interface = utils::getParameter<std::string>(nodeHandle, "phys1_interface", "can1");
    phys2Interface = utils::getParameter<std::string>(nodeHandle, "phys2_interface", "can2");

    // Number of motors in this robot configuration (5 or 6)
    int numMotorsParam = utils::getParameter<int>(nodeHandle, "num_motors", 6);
    if (numMotorsParam < 1 || numMotorsParam > (int)MAX_MOTORS) {
        RCLCPP_FATAL(nodeHandle->get_logger(),
            "num_motors must be between 1 and %d, got %d", (int)MAX_MOTORS, numMotorsParam);
        return 1;
    }
    numMotors = (size_t)numMotorsParam;

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

    RCLCPP_INFO(nodeHandle->get_logger(),
        "CAN interfaces: vcan=%s, phys1=%s, phys2=%s",
        vcanInterface.c_str(), phys1Interface.c_str(), phys2Interface.c_str());

    // Motor CAN IDs in physical wiring order (closest to phys1 first).
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

    canIdToWiringIndex.clear();
    for (size_t i = 0; i < numMotors; ++i) {
        canIdToWiringIndex[motorWiringOrder[i]] = i;
        RCLCPP_INFO(nodeHandle->get_logger(),
            "Wiring position %zu: motor CAN ID 0x%X", i, motorWiringOrder[i]);
    }

    for (size_t i = 0; i < MAX_MOTORS; i++) {
        motors1[i] = 0;
        motors2[i] = 0;
        copy1[i] = 0;
        copy2[i] = 0;
        interfaces[i] = -1;
    }

    getInterfaceName();

    std::thread can1_thread(can_read_loop, phys1Interface, std::ref(motors1), std::ref(mutex1));
    std::thread can2_thread(can_read_loop, phys2Interface, std::ref(motors2), std::ref(mutex2));

    rclcpp::Rate rate(20);
    while (rclcpp::ok()) {
        rclcpp::spin_some(nodeHandle);
        statusCheck();
        rate.sleep();
    }

    run_threads = false;
    if (can1_thread.joinable()) can1_thread.join();
    if (can2_thread.joinable()) can2_thread.join();

    rclcpp::shutdown();
    return 0;
}