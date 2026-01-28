#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <set>

#include "MockDeps.hpp"
#include "Heartbeat.hpp"
#include "CANHeartbeat.hpp"
#include "EthernetHBThread.hpp"

#include "AegisController.hpp"
#include "AegisNanoController.hpp"

// Ports for localhost testing
#define ORIN_PORT 31337
#define NANO_PORT 31338
#define LOCAL_IP "127.0.0.1"

class SystemIntegrationTest : public ::testing::Test {
protected:
    std::shared_ptr<rclcpp::Node> orinNode;
    std::unique_ptr<HeartbeatLink> orinLink;
    std::unique_ptr<CanLink> orinCanLink;
    std::shared_ptr<AegisController> orinController;
    std::mutex orinMutex;
    RemoteStatus orinRemoteStatus;
    bool orinRawData = false;
    
    // Initialize to BOOT so Watchdog doesn't fire immediately
    SystemStatus orinSysStatus = BOOT; 
    HandshakeStatus orinHandshakeStatus = IDLE_HANDSHAKE;
    ErrorCode orinErrorCode = NO_ERROR;

    std::shared_ptr<rclcpp::Node> nanoNode;
    std::unique_ptr<HeartbeatLink> nanoLink;
    std::unique_ptr<CanLink> nanoCanLink;
    std::shared_ptr<AegisNanoController> nanoController;
    std::mutex nanoMutex;
    RemoteStatus nanoRemoteStatus;
    bool nanoRawData = false;
    
    // Initialize to BOOT
    SystemStatus nanoSysStatus = BOOT;
    HandshakeStatus nanoHandshakeStatus = IDLE_HANDSHAKE;
    ErrorCode nanoErrorCode = NO_ERROR;

    std::atomic<bool> orin_running {true};
    std::atomic<bool> nano_running {true};
    std::thread nano_thread;
    std::thread orin_thread;

    std::unique_ptr<EthernetHBThread> orinEthHB;
    std::unique_ptr<EthernetHBThread> nanoEthHB;

    CanHeartbeatPayload orin_hb {0x01, 0, 0, 0};
    CanHeartbeatPayload nano_hb {0x02, 0, 0, 0};

    void SetUp() override {
        // 1. Setup Orin (Sends to Nano)
        orinNode = rclcpp::Node::make_shared("orin_node");
        orinLink = std::make_unique<HeartbeatLink>(ORIN_PORT, LOCAL_IP, NANO_PORT);
        orinLink->init();
        orinCanLink = std::make_unique<CanLink>();
        orinController = std::make_shared<AegisController>(
            orinNode, *orinLink, *orinCanLink, orinMutex, orinRemoteStatus, orinRawData, orinSysStatus, orinHandshakeStatus, orinErrorCode
        );
        using namespace std::placeholders;
        orinLink->set_data_callback(
            std::bind(&AegisController::on_packet_received, orinController, _1, _2, _3)
        );
        orinEthHB = std::make_unique<EthernetHBThread>(*orinLink, std::chrono::milliseconds(10));

        // 2. Setup Nano (Sends to Orin)
        nanoNode = rclcpp::Node::make_shared("nano_node");
        nanoLink = std::make_unique<HeartbeatLink>(NANO_PORT, LOCAL_IP, ORIN_PORT);
        nanoLink->init();
        nanoCanLink = std::make_unique<CanLink>();
        nanoController = std::make_shared<AegisNanoController>(
            nanoNode, *nanoLink, *nanoCanLink, nanoMutex, nanoRemoteStatus, nanoRawData, nanoSysStatus, nanoHandshakeStatus, nanoErrorCode
        );
        nanoLink->set_data_callback(
            std::bind(&AegisNanoController::on_packet_received, nanoController, _1, _2, _3)
        );
        nanoEthHB = std::make_unique<EthernetHBThread>(*nanoLink, std::chrono::milliseconds(10));

        // Start the boot timers
        orinController->initAegis();
        nanoController->initAegis();

        // 3. Initialize Motors
        InitializeAllMotors();

        // 4. Start Communication Threads
        orin_thread = std::thread([this]() {
            while (orin_running) {
                while(orinLink->spin_once());
                orinController->checkTimers();
                orinCanLink->read_heartbeat(nano_hb);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });

        orinEthHB->start();

        // Nano thread
        nano_thread = std::thread([this]() {
             while (nano_running) { 
                while(nanoLink->spin_once());
                nanoController->checkTimers();
                nanoCanLink->read_heartbeat(orin_hb);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });

        nanoEthHB->start();

        // 5. Wait for connection to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    void TearDown() override {
        orin_running = false;
        nano_running = false;
        if (orin_thread.joinable()) orin_thread.join();
        if (nano_thread.joinable()) nano_thread.join();
        orinLink->close_socket();
        nanoLink->close_socket();

        orinEthHB->stop();
        nanoEthHB->stop();
    }

    void startOrinThread() {
        orin_running = true;
        orinEthHB->start();
        orin_thread = std::thread([this]() {
            while (orin_running) {
                while(orinLink->spin_once());
                orinController->checkTimers();
                orinCanLink->read_heartbeat(nano_hb);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    void startNanoThread() {
        nano_running = true;
        nanoEthHB->start();
        nano_thread = std::thread([this]() {
             while (nano_running) { 
                while(nanoLink->spin_once());
                nanoController->checkTimers();
                nanoCanLink->read_heartbeat(orin_hb);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    void InitializeAllMotors() {
        for (int id = 10; id <= 17; id++) {
            // Set Orin Motors
            orinController->updateMotorCAN0State(id, true);
            orinController->updateMotorCAN1State(id, true);

            nanoController->updateMotorCAN0State(id, true);
            nanoController->updateMotorCAN1State(id, true);
        }
    }

    void RemoveMotors(int num_motors){
        for (int id = 10; id < 10 + num_motors; id++) {
            // Set Orin Motors
            orinController->updateMotorCAN0State(id, false);
            orinController->updateMotorCAN1State(id, false);

            nanoController->updateMotorCAN0State(id, false);
            nanoController->updateMotorCAN1State(id, false);
        }
    }

    void ValidateFlexibleFlow(const std::string& captured_logs, const std::vector<std::vector<std::string>>& expected_groups) {
        std::stringstream ss(captured_logs);
        std::string line;
        std::vector<std::string> actual_sequence;

        while (std::getline(ss, line)) {
            if (line.find("Received Message ID:") != std::string::npos) {
                std::string cleaned;
                if (line.find("Orin:") != std::string::npos)      cleaned = "Orin -> ";
                else if (line.find("Nano:") != std::string::npos) cleaned = "Nano -> ";
                else continue; 

                size_t id_pos = line.find("ID: ");
                if (id_pos != std::string::npos) {
                    cleaned += line.substr(id_pos + 4);
                    actual_sequence.push_back(cleaned);
                }
            }
        }

        int actual_index = 0;
        bool mismatch = false;
        std::string failure_reason;

        for (size_t g = 0; g < expected_groups.size(); g++) {
            const auto& group = expected_groups[g];
            int group_size = group.size();

            if (actual_index + group_size > actual_sequence.size()) {
                failure_reason = "Premature end of log. Waiting for Group " + std::to_string(g + 1);
                mismatch = true;
                break;
            }

            std::vector<std::string> actual_batch;
            for (int i = 0; i < group_size; i++) {
                actual_batch.push_back(actual_sequence[actual_index + i]);
            }

            if (group_size == 1) {
                if (actual_batch[0] != group[0]) {
                    failure_reason = "Mismatch at Step " + std::to_string(actual_index + 1);
                    mismatch = true;
                    break;
                }
            } else {
                std::multiset<std::string> expected_set(group.begin(), group.end());
                std::multiset<std::string> actual_set(actual_batch.begin(), actual_batch.end());

                if (expected_set != actual_set) {
                    failure_reason = "Set Mismatch in Group " + std::to_string(g + 1);
                    mismatch = true;
                    break;
                }
            }
            actual_index += group_size;
        }

        if (mismatch) {
            std::cout << "\n=== PACKET FLOW FAILURE: " << failure_reason << " ===\n";
            FAIL() << "Flexible Packet Flow deviation detected.";
        }
    }
};

TEST_F(SystemIntegrationTest, NanoShutdown){
    nanoSysStatus = STANDBY; 
    orinSysStatus = PRIMARY; // Force Primary for this test scenario

    std::cout << "[TEST] Injecting ID 500 (System Shutdown)" << std::endl;
    nanoLink->send_data(500, "", 0);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_EQ(orinSysStatus, SINGLE_FC) << "Orin did not enter SINGLE_FC as expected";
}

TEST_F(SystemIntegrationTest, NanoReboot){
    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "[TEST] Injecting ID 500 (System Shutdown)" << std::endl;
    nanoLink->send_data(500, "", 0);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_EQ(orinSysStatus, SINGLE_FC) << "Orin did not enter SINGLE_FC as expected";

    std::cout << "[TEST] Injecting ID 501 (System Shutdown)" << std::endl;
    nanoLink->send_data(501, "", 0);
    
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for 1s boot timer + handshake
    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
}

TEST_F(SystemIntegrationTest, CanParserTest) {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    uint32_t id = 0;
    id |= (10 & 0x1F) << 24; 
    id |= (15 & 0xFF) << 16; 
    id |= (1  & 0x3F) << 10; 
    id |= (1  & 0x0F) << 6;  
    id |= (2  & 0x3F) << 0;  
    
    frame.can_id = id | CAN_EFF_FLAG; 
    frame.can_dlc = sizeof(CanHeartbeatPayload);

    CanHeartbeatPayload tx_payload;
    tx_payload.seq_counter = 42;
    tx_payload.system_status = PRIMARY;
    tx_payload.system_flags = 0xFF;
    std::memcpy(frame.data, &tx_payload, sizeof(tx_payload));

    CanHeartbeatPayload rx_payload;
    bool success = orinCanLink->parse_heartbeat(frame, rx_payload);

    ASSERT_TRUE(success) << "Failed to parse valid Heartbeat frame";
    EXPECT_EQ(rx_payload.seq_counter, 42);
    EXPECT_EQ(rx_payload.system_status, PRIMARY);
    EXPECT_EQ(rx_payload.system_flags, 0xFF);
}

TEST_F(SystemIntegrationTest, CanDataFailoverTest) {
    // Force SINGLE_FC so processing happens without active HB
    orinSysStatus = SINGLE_FC; 

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    uint32_t id = 0;
    id |= (10 & 0x1F) << 24; 
    id |= (15 & 0xFF) << 16; 
    id |= (2  & 0x3F) << 10; 
    id |= (2  & 0x0F) << 6;  
    id |= (2  & 0x3F) << 0;  
    frame.can_id = id | CAN_EFF_FLAG;
    frame.can_dlc = sizeof(CanDataPayload);

    CanDataPayload data_load;
    data_load.message_id = ID_JAXIS_MSG;
    JoystickAxis joyMsg {0, 0, 1.0f}; 
    std::memcpy(data_load.data, &joyMsg, sizeof(joyMsg));
    std::memcpy(frame.data, &data_load, sizeof(data_load));

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Check using accessor or helper since is_remote_alive() might flicker if we don't wait long enough
    // But Orin watchdog in bg thread should have set remoteStatus.UP = false

    CanDataPayload parsed_data;
    ASSERT_TRUE(orinCanLink->parse_data(frame, parsed_data));
    orinController->onCanDataReceived(parsed_data); // Direct call safe here

    std::cout << "[TEST] Killing Ethernet..." << std::endl;
    orin_running = false; 
    // Wait for thread termination handled in TearDown, or manually join here if needed to be sure
    // But we need to simulate connection loss.
    // The previous test logic just waited.
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    
    ASSERT_FALSE(orinLink->is_remote_alive());

    std::cout << "[TEST] Injecting CAN Joystick Command..." << std::endl;
    orinController->onCanDataReceived(parsed_data);
}

TEST_F(SystemIntegrationTest, NormalStartSequence){
    nanoSysStatus = BOOT; 
    orinSysStatus = BOOT;
    
    // Let threads establish connection
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "[TEST] Injecting ID 501 (System Booted)" << std::endl;
    // Calling initAegis starts the 1s timer. Background threads handle ticks.
    orinController->initAegis();
    nanoController->initAegis();

    // Wait for 1s boot + handshake time
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    
    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";
}


TEST_F(SystemIntegrationTest, AbnormalStart_NanoAbsent){
    // Kill Nano thread simulation
    nano_running = false; 
    if (nano_thread.joinable()) nano_thread.join();
    nanoEthHB->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    orinController->initAegis();
    
    // Wait for boot timer (1s) + Watchdog timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));

    ASSERT_EQ(orinSysStatus, SINGLE_FC) << "Orin did not enter SINGLE_FC as expected";
}

TEST_F(SystemIntegrationTest, AbnormalStart_OrinAbsent){
    orin_running = false; 
    if (orin_thread.joinable()) orin_thread.join();
    orinEthHB->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    nanoController->initAegis();

    // Wait for boot timer (1s) + Watchdog timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));

    ASSERT_EQ(nanoSysStatus, SINGLE_FC) << "Nano did not enter SINGLE_FC as expected";
}

TEST_F(SystemIntegrationTest, AbnormalStart_NanoDelayed){
    nano_running = false;
    if (nano_thread.joinable()) nano_thread.join();
    nanoEthHB->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    orinSysStatus = BOOT;
    orinController->initAegis();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    ASSERT_EQ(orinSysStatus, SINGLE_FC) << "Orin did not enter SINGLE_FC as expected";

    std::cout << "[TEST] Nano Booting Late..." << std::endl;
    
    nanoSysStatus = BOOT;
    startNanoThread();
    nanoController->initAegis();

    std::this_thread::sleep_for(std::chrono::milliseconds(2500));

    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";
}

TEST_F(SystemIntegrationTest, AbnormalStart_OrinDelayed){
    orin_running = false;
    if (orin_thread.joinable()) orin_thread.join();
    orinEthHB->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    nanoSysStatus = BOOT;
    nanoController->initAegis();

    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    ASSERT_EQ(nanoSysStatus, SINGLE_FC) << "Nano did not enter SINGLE_FC as expected";
    std::cout << "[TEST] Orin Booting Late..." << std::endl;
    
    orinSysStatus = BOOT;
    startOrinThread();
    orinController->initAegis();

    std::this_thread::sleep_for(std::chrono::milliseconds(2500));

    ASSERT_EQ(nanoSysStatus, PRIMARY) << "Nano did not enter PRIMARY as expected";
    ASSERT_EQ(orinSysStatus, STANDBY) << "Orin did not enter STANDBY as expected";
}

TEST_F(SystemIntegrationTest, NormalStartSequence_NoMotors){
    nanoSysStatus = BOOT; 
    orinSysStatus = BOOT;

    RemoveMotors(8);
    
    // Let threads establish connection
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "[TEST] Injecting ID 501 (System Booted)" << std::endl;
    // Calling initAegis starts the 1s timer. Background threads handle ticks.
    orinController->initAegis();
    nanoController->initAegis();

    // Wait for 1s boot + handshake time
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    
    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";
}

TEST_F(SystemIntegrationTest, AbnormalStart_NanoDelayed_NoMotors){
    nano_running = false;
    if (nano_thread.joinable()) nano_thread.join();
    nanoEthHB->stop();

    RemoveMotors(8);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    orinSysStatus = BOOT;
    orinController->initAegis();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    ASSERT_EQ(orinSysStatus, SINGLE_FC) << "Orin did not enter SINGLE_FC as expected";

    std::cout << "[TEST] Nano Booting Late..." << std::endl;
    
    nanoSysStatus = BOOT;
    startNanoThread();
    nanoController->initAegis();

    std::this_thread::sleep_for(std::chrono::milliseconds(2500));

    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";
}

TEST_F(SystemIntegrationTest, AbnormalStart_OrinDelayed_NoMotors){
    orin_running = false;
    if (orin_thread.joinable()) orin_thread.join();
    orinEthHB->stop();

    RemoveMotors(8);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    nanoSysStatus = BOOT;
    nanoController->initAegis();

    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    ASSERT_EQ(nanoSysStatus, SINGLE_FC) << "Nano did not enter SINGLE_FC as expected";
    std::cout << "[TEST] Orin Booting Late..." << std::endl;
    
    orinSysStatus = BOOT;
    startOrinThread();
    orinController->initAegis();

    std::this_thread::sleep_for(std::chrono::milliseconds(2500));

    ASSERT_EQ(nanoSysStatus, PRIMARY) << "Nano did not enter PRIMARY as expected";
    ASSERT_EQ(orinSysStatus, STANDBY) << "Orin did not enter STANDBY as expected";
}

/*
// Partial Peer Heartbeat: No CAN - Nano
TEST_F(SystemIntegrationTest, AbnormalStart_No_CAN_HB_From_Nano){
    std::cout << "TODO" << std::endl;
}

// Partial Peer Heartbeat: No CAN - Orin
TEST_F(SystemIntegrationTest, AbnormalStart_No_CAN_HB_From_Orin){
    std::cout << "TODO" << std::endl;
}

// Partial Peer Heartbeat: No Ethernet - Nano
TEST_F(SystemIntegrationTest, AbnormalStart_No_Eth_HB_From_Nano){
    std::cout << "TODO" << std::endl;
}

// Partial Peer Heartbeat: No Ethernet - Orin
TEST_F(SystemIntegrationTest, AbnormalStart_No_Eth_HB_From_Orin){
    std::cout << "TODO" << std::endl;
}

// Orin Does Not Take Control
TEST_F(SystemIntegrationTest, AbnormalStart_Orin_Does_Not_Take_Charge){
    nanoSysStatus = STANDBY; 
    orinSysStatus = ERROR;
    
    for (int i=0; i<10; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[TEST] Injecting ID 501 (System Booted)" << std::endl;
    nanoLink->send_data(501, "", 0);
    orinLink->send_data(501, "", 0);
    orinLink->send_data(202, "", 0);
    
    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(orinSysStatus, ERROR) << "Orin did not enter ERROR as expected";
    ASSERT_EQ(nanoSysStatus, PRIMARY) << "Nano did not enter PRIMARY as expected";
}


TEST_F(SystemIntegrationTest, NormalStartSequenceWithMotorInit){
    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;

    orinController->alertSystemBoot();
    nanoController->alertSystemBoot();

    for (int i=0; i<10; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int i=0; i<10; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    orinController->alertMotorsDetected();

    for (int i=0; i<10; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";
}

TEST_F(SystemIntegrationTest, NormalStartSequenceWithMissingMotor_OrinPrimary){
    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;
    
    orinController->updateMotorCAN0State(10, false);
    orinController->updateMotorCAN1State(10, false);

    orinController->alertSystemBoot();
    nanoController->alertSystemBoot();

    for (int i=0; i<10; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int i=0; i<10; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    orinController->alertMotorsDetected();

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(orinSysStatus, PARTIAL_PRIMARY) << "Orin did not enter PARTIAL_PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, PARTIAL_SECONDARY) << "Nano did not enter PARTIAL_SECONDARY as expected";
}

// This is designed to test whether FC2 will successfully relinquish control
TEST_F(SystemIntegrationTest, NormalStartSequenceWithMissingMotor_OrinPrimary_DNR){
    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;

    orinController->alertSystemBoot();
    nanoController->alertSystemBoot();

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";
    std::cout << "Lost motor" << std::endl;
    orinController->alertLostMotor(10);

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(orinSysStatus, PARTIAL_PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, PARTIAL_SECONDARY) << "Nano did not enter STANDBY as expected";

    std::cout << "Regained motor" << std::endl;
    orinController->test = true;
    orinController->alertRegainedMotor(10);
    for (int i=0; i<40; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";
}

TEST_F(SystemIntegrationTest, NormalStartSequenceWithMissingMotorFromBoth){
    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;
    
    orinController->updateMotorCAN0State(10, false);
    orinController->updateMotorCAN1State(10, false);
    nanoController->updateMotorCAN0State(10, false);
    nanoController->updateMotorCAN1State(10, false);

    orinController->alertSystemBoot();
    nanoController->alertSystemBoot();

    for (int i=0; i<10; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int i=0; i<10; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    orinController->alertMotorsDetected();

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(orinSysStatus, STOP) << "Orin did not enter STOP as expected";
    ASSERT_EQ(nanoSysStatus, STOP) << "Nano did not enter STOP as expected";
}

*/
// --- Motor Node Crashes ---
/* Tested configurations:
 * Orin Primary, Nano Primary
 * Orin SINGLE_FC, Nano SINGLE_FC
*/
/*
// Orin Primary
TEST_F(SystemIntegrationTest, MotorNodeCrash_OrinPrimary){
    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;

    orinController->alertSystemBoot();
    nanoController->alertSystemBoot();

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";
    std::cout << "Lost motor" << std::endl;
    orinController->alertLostMotor(10);

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(orinSysStatus, PARTIAL_PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, PARTIAL_SECONDARY) << "Nano did not enter STANDBY as expected";

    std::cout << "Regained motor" << std::endl;
    orinController->alertRegainedMotor(10);

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";
}

TEST_F(SystemIntegrationTest, MotorNodeCrashNano_OrinPrimary){
    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;

    orinController->alertSystemBoot();
    nanoController->alertSystemBoot();

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";
    std::cout << "Lost motor" << std::endl;
    nanoController->alertLostMotor(10);

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";

    std::cout << "Regained motor" << std::endl;
    nanoController->alertRegainedMotor(10);

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";
}

// Nano Primary
TEST_F(SystemIntegrationTest, MotorNodeCrash_NanoPrimary){
    std::cout << "TODO" << std::endl;
}

// Orin SINGLE_FC
TEST_F(SystemIntegrationTest, MotorNodeCrash_Orin_Single){
    nano_running = false;
    if (nano_thread.joinable()) {
        nano_thread.join();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    orinSysStatus = STANDBY;

    orinController->alertSystemBoot();
    orinController->queryControl();

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(orinSysStatus, SINGLE_FC) << "Orin did not enter SINGLE_FC as expected";
    std::cout << "Lost motor" << std::endl;
    orinController->alertLostMotor(10);

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(orinSysStatus, STOP) << "Orin did not enter STOP as expected";

    std::cout << "Regained motor" << std::endl;
    orinController->alertRegainedMotor(10);

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    ASSERT_EQ(orinSysStatus, SINGLE_FC) << "Orin did not enter SINGLE_FC as expected";
}

// Nano SINGLE_FC
TEST_F(SystemIntegrationTest, MotorNodeCrash_Nano_Single){
    orin_running = false;
    if (orin_thread.joinable()) {
        orin_thread.join();
    }
    while(orinLink->spin_once()); 

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    nanoSysStatus = STANDBY; 
    nanoController->alertSystemBoot();
    nanoController->queryControl();

    for (int i=0; i<20; i++) {
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(nanoSysStatus, SINGLE_FC) << "Nano did not enter SINGLE_FC as expected";
    std::cout << "Lost motor" << std::endl;
    nanoController->alertLostMotor(10);

    for (int i=0; i<20; i++) {
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(nanoSysStatus, STOP) << "Nano did not enter STOP as expected";

    std::cout << "Regained motor" << std::endl;
    nanoController->alertRegainedMotor(10);

    for (int i=0; i<20; i++) {
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    ASSERT_EQ(nanoSysStatus, SINGLE_FC) << "Nano did not enter SINGLE_FC as expected";
}

TEST_F(SystemIntegrationTest, MotorNodeCrash_Orin_Single_Nano_Rejoins_During_Reboot){
    orin_running = false;
    if (orin_thread.joinable()) {
        orin_thread.join();
    }
}

TEST_F(SystemIntegrationTest, MotorNodeCrash_Nano_Single_Orin_Rejoins_During_Reboot){
    orin_running = false;
    if (orin_thread.joinable()) {
        orin_thread.join();
    }
}

TEST_F(SystemIntegrationTest, MotorNodeCrash_Orin_Single_Nano_Rejoins_After_Reboot){
    orin_running = false;
    if (orin_thread.joinable()) {
        orin_thread.join();
    }
}

TEST_F(SystemIntegrationTest, MotorNodeCrash_Nano_Single_Orin_Rejoins_After_Reboot){
    orin_running = false;
    if (orin_thread.joinable()) {
        orin_thread.join();
    }
}

*/
/*
TEST_F(SystemIntegrationTest, PingNano){
    testing::internal::CaptureStdout();

    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;

    orinController->alertSystemBoot();
    nanoController->alertSystemBoot();

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";

    orinController->sendPing();

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";
    std::string output = testing::internal::GetCapturedStdout();
    std::cout << output;

    std::vector<std::vector<std::string>> expected_flow = {
        {"Orin -> 501", "Nano -> 501"}, 
        {"Nano -> 200", "Orin -> 200"}, 
        {"Nano -> 202", "Orin -> 202"}, 
        {"Nano -> 211"}, 
        {"Orin -> 212"},
        {"Nano -> 206"},
        {"Orin -> 207"}
    };

    ValidateFlexibleFlow(output, expected_flow);
}

TEST_F(SystemIntegrationTest, PingOrin){
    testing::internal::CaptureStdout();

    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;

    orinController->alertSystemBoot();
    nanoController->alertSystemBoot();

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";

    nanoController->sendPing();

    for (int i=0; i<20; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";

    std::string output = testing::internal::GetCapturedStdout();
    std::cout << output;

    std::vector<std::vector<std::string>> expected_flow = {
        {"Orin -> 501", "Nano -> 501"}, 
        {"Nano -> 200", "Orin -> 200"}, 
        {"Nano -> 202", "Orin -> 202"}, 
        {"Nano -> 211"}, 
        {"Orin -> 212"},
        {"Orin -> 206"},
        {"Nano -> 207"}
    };

    ValidateFlexibleFlow(output, expected_flow);
}
    */
    
   