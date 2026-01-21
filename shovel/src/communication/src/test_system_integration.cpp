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
    SystemStatus orinSysStatus = PRIMARY;
    ErrorCode orinErrorCode = NO_ERROR;

    std::shared_ptr<rclcpp::Node> nanoNode;
    std::unique_ptr<HeartbeatLink> nanoLink;
    std::unique_ptr<CanLink> nanoCanLink;
    std::shared_ptr<AegisNanoController> nanoController;
    std::mutex nanoMutex;
    RemoteStatus nanoRemoteStatus;
    bool nanoRawData = false;
    SystemStatus nanoSysStatus = STANDBY;
    ErrorCode nanoErrorCode = NO_ERROR;

    std::atomic<bool> orin_running {true};
    std::atomic<bool> orin_can_running{true};
    std::atomic<bool> orin_eth_running{true};
    std::atomic<bool> nano_running {true};
    std::atomic<bool> nano_can_running{true};
    std::atomic<bool> nano_eth_running{true};
    std::thread nano_thread;
    std::thread orin_thread;

    CanHeartbeatPayload orin_hb {0x01, 0, 0, 0};
    CanHeartbeatPayload nano_hb {0x02, 0, 0, 0};

    void SetUp() override {
        // 1. Setup Orin (Sends to Nano)
        orinNode = rclcpp::Node::make_shared("orin_node");
        orinLink = std::make_unique<HeartbeatLink>(ORIN_PORT, LOCAL_IP, NANO_PORT);
        orinLink->init();
        orinCanLink = std::make_unique<CanLink>();
        orinController = std::make_shared<AegisController>(
            orinNode, *orinLink, *orinCanLink, orinMutex, orinRemoteStatus, orinRawData, orinSysStatus, orinErrorCode
        );
        using namespace std::placeholders;
        orinLink->set_data_callback(
            std::bind(&AegisController::on_packet_received, orinController, _1, _2, _3)
        );

        // 2. Setup Nano (Sends to Orin)
        nanoNode = rclcpp::Node::make_shared("nano_node");
        nanoLink = std::make_unique<HeartbeatLink>(NANO_PORT, LOCAL_IP, ORIN_PORT);
        nanoLink->init();
        nanoCanLink = std::make_unique<CanLink>();
        nanoController = std::make_shared<AegisNanoController>(
            nanoNode, *nanoLink, *nanoCanLink, nanoMutex, nanoRemoteStatus, nanoRawData, nanoSysStatus, nanoErrorCode
        );
        nanoLink->set_data_callback(
            std::bind(&AegisNanoController::on_packet_received, nanoController, _1, _2, _3)
        );

        // 3. Start Communication Threads
        orin_thread = std::thread([this]() {
            while (orin_running) {
                orinLink->spin_once();
                orinLink->send_heartbeat();
                orinController->checkTimers();
                orinCanLink->read_heartbeat(nano_hb);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        nano_thread = std::thread([this]() {
            while (nano_running) {
                nanoLink->spin_once();
                nanoController->checkTimers();
                nanoLink->send_heartbeat();
                nanoCanLink->read_heartbeat(orin_hb);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        // 4. Wait for connection to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        ASSERT_TRUE(orinLink->is_remote_alive()) << "Orin cannot see Nano";
        ASSERT_TRUE(nanoLink->is_remote_alive()) << "Nano cannot see Orin";

        InitializeAllMotors();
    }

    void TearDown() override {
        orin_running = false;
        nano_running = false;
        if (orin_thread.joinable()) orin_thread.join();
        if (nano_thread.joinable()) nano_thread.join();
        orinLink->close_socket();
        nanoLink->close_socket();
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

    std::string cleanLogLine(const std::string& line) {
        std::string cleaned;
        if (line.find("Orin:") != std::string::npos)      cleaned = "Orin -> ";
        else if (line.find("Nano:") != std::string::npos) cleaned = "Nano -> ";
        else return "";

        size_t id_pos = line.find("ID: ");
        if (id_pos != std::string::npos) {
            cleaned += line.substr(id_pos + 4);
        }
        return cleaned;
    }

    void ValidateFlexibleFlow(const std::string& captured_logs, const std::vector<std::vector<std::string>>& expected_groups) {
        std::stringstream ss(captured_logs);
        std::string line;
        std::vector<std::string> actual_sequence;

        // --- 1. PARSE LOGS ---
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

        // --- 2. VALIDATE LOGIC ---
        int actual_index = 0;
        bool mismatch = false;
        std::string failure_reason;

        for (size_t g = 0; g < expected_groups.size(); g++) {
            const auto& group = expected_groups[g];
            int group_size = group.size();

            // Check for Premature End
            if (actual_index + group_size > actual_sequence.size()) {
                failure_reason = "Premature end of log. Waiting for Group " + std::to_string(g + 1);
                mismatch = true;
                break;
            }

            // Get Batch
            std::vector<std::string> actual_batch;
            for (int i = 0; i < group_size; i++) {
                actual_batch.push_back(actual_sequence[actual_index + i]);
            }

            // Validate Batch
            if (group_size == 1) {
                // Strict Order
                if (actual_batch[0] != group[0]) {
                    failure_reason = "Mismatch at Step " + std::to_string(actual_index + 1);
                    mismatch = true;
                    break; // Stop validation to print report
                }
            } else {
                // Loose Order (Set Comparison)
                std::multiset<std::string> expected_set(group.begin(), group.end());
                std::multiset<std::string> actual_set(actual_batch.begin(), actual_batch.end());

                if (expected_set != actual_set) {
                    failure_reason = "Set Mismatch in Group " + std::to_string(g + 1);
                    mismatch = true;
                    break; // Stop validation to print report
                }
            }
            actual_index += group_size;
        }

        // Check for extra trailing packets (Optional strictness)
        if (!mismatch && actual_index < actual_sequence.size()) {
            failure_reason = "Extra packets detected at end of log.";
            mismatch = true;
        }

        // --- 3. DETAILED REPORTING ---
        if (mismatch) {
            std::cout << "\n=======================================\n";
            std::cout << "      PACKET FLOW FAILURE: " << failure_reason << "\n";
            std::cout << "=======================================\n";

            std::cout << "\n--- EXPECTED FLOW (GROUPS) ---\n";
            for (size_t i = 0; i < expected_groups.size(); i++) {
                std::cout << "Group " << (i + 1) << ": { ";
                for (size_t j = 0; j < expected_groups[i].size(); j++) {
                    std::cout << "\"" << expected_groups[i][j] << "\"";
                    if (j < expected_groups[i].size() - 1) std::cout << ", ";
                }
                std::cout << " }\n";
            }

            std::cout << "\n--- ACTUAL FLOW (RECEIVED) ---\n";
            for (size_t i = 0; i < actual_sequence.size(); i++) {
                // Highlight the point where validation stopped/failed
                std::string prefix = (i >= actual_index) ? ">>> " : "    ";
                std::cout << prefix << "[Step " << (i + 1) << "] " << actual_sequence[i] << "\n";
            }
            
            // If we ran out of logs, show where the next step WOULD have been
            if (actual_index >= actual_sequence.size()) {
                std::cout << ">>> [Step " << (actual_sequence.size() + 1) << "] (MISSING)\n";
            }

            std::cout << "---------------------------------------\n";
            FAIL() << "Flexible Packet Flow deviation detected.";
        }
    }

    bool CheckScenario(const std::vector<std::string>& actual_sequence, 
                    const std::vector<std::vector<std::string>>& scenario, 
                    std::string& failure_reason) {
        int actual_index = 0;

        for (size_t g = 0; g < scenario.size(); g++) {
            const auto& group = scenario[g];
            int group_size = group.size();

            if (actual_index + group_size > actual_sequence.size()) {
                failure_reason = "Premature end of log. Waiting for Group " + std::to_string(g + 1);
                return false;
            }

            std::vector<std::string> actual_batch;
            for (int i = 0; i < group_size; i++) {
                actual_batch.push_back(actual_sequence[actual_index + i]);
            }

            if (group_size == 1) {
                if (actual_batch[0] != group[0]) {
                    failure_reason = "Mismatch at Group " + std::to_string(g + 1) + 
                                    " (Expected " + group[0] + ", Got " + actual_batch[0] + ")";
                    return false;
                }
            } else {
                std::multiset<std::string> expected_set(group.begin(), group.end());
                std::multiset<std::string> actual_set(actual_batch.begin(), actual_batch.end());

                if (expected_set != actual_set) {
                    failure_reason = "Set Mismatch in Group " + std::to_string(g + 1);
                    return false;
                }
            }
            actual_index += group_size;
        }

        if (actual_index < actual_sequence.size()) {
            failure_reason = "Extra packets detected at end of log.";
            return false;
        }

        return true;
    }

    void ValidateMultiScenarioFlow(const std::string& captured_logs, 
                                const std::vector<std::vector<std::vector<std::string>>>& valid_scenarios) {
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

        std::vector<std::string> reasons;
        for (size_t i = 0; i < valid_scenarios.size(); i++) {
            std::string reason;
            if (CheckScenario(actual_sequence, valid_scenarios[i], reason)) {
                return;
            }
            reasons.push_back("Scenario " + std::to_string(i + 1) + ": " + reason);
        }

        std::cout << "\n=======================================\n";
        std::cout << "      PACKET FLOW FAILURE              \n";
        std::cout << "=======================================\n";
        std::cout << "None of the valid scenarios matched the actual flow.\n";
        
        std::cout << "\n--- ACTUAL FLOW ---\n";
        for (size_t i = 0; i < actual_sequence.size(); i++) {
            std::cout << "[" << (i + 1) << "] " << actual_sequence[i] << "\n";
        }

        std::cout << "\n--- FAILURE REASONS ---\n";
        for (const auto& r : reasons) std::cout << r << "\n";
        std::cout << "---------------------------------------\n";
        
        FAIL() << "Packet Flow did not match any valid scenario.";
    }

};

TEST_F(SystemIntegrationTest, EstablishHeartbeatConnection) {
    // Run for 100ms to allow heartbeats to exchange
    for (int i = 0; i < 10; i++) {
        orinLink->spin_once();
        orinLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Verify both sides see each other
    EXPECT_TRUE(orinLink->is_remote_alive()) << "Orin should see Nano";
    EXPECT_TRUE(nanoLink->is_remote_alive()) << "Nano should see Orin";
}

TEST_F(SystemIntegrationTest, OrinQuerysNanoControl_OrinPrimary) {
    testing::internal::CaptureStdout();
    
    // 1. Send control query from Orin to Nano
    std::cout << "[TEST] Query Control" << std::endl;
    orinController->queryControl();

    for (int i = 0; i < 10; i++) {
        orinLink->spin_once();
        orinLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_TRUE(orinSysStatus == PRIMARY) << "Orin should be PRIMARY";
    std::string output = testing::internal::GetCapturedStdout();
    std::cout << output;

    std::vector<std::vector<std::string>> expected_flow = {
        {"Nano -> 200"}, 
        {"Orin -> 202"}, 
    };

    ValidateFlexibleFlow(output, expected_flow);
}

TEST_F(SystemIntegrationTest, OrinQuerysNanoControl_OrinStandby) {
    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;
    
    for (int i=0; i<10; i++) {
        orinLink->spin_once(); orinLink->send_heartbeat();
        nanoLink->spin_once(); nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[TEST] Query Control" << std::endl;
    orinController->queryControl();

    for (int i = 0; i < 20; i++) {
        orinLink->spin_once();
        nanoLink->spin_once();
        orinLink->send_heartbeat();
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "orinSysStatus: " << (int)orinSysStatus << std::endl;
    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not switch to PRIMARY after Nano yielded control";
}

TEST_F(SystemIntegrationTest, OrinQuerysNanoControl_AlertAck){
    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;
    
    for (int i=0; i<10; i++) {
        orinLink->spin_once(); orinLink->send_heartbeat();
        nanoLink->spin_once(); nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[TEST] Query Control" << std::endl;
    orinController->queryControl();

    for (int i = 0; i < 20; i++) {
        orinLink->spin_once();
        nanoLink->spin_once();
        orinLink->send_heartbeat();
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    uint8_t error_val = 1;
    
    std::cout << "[TEST] Injecting ID 212 (System Error)..." << std::endl;
    nanoLink->send_data(212, &error_val, sizeof(error_val));

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    orinLink->spin_once();
    std::cout << "TODO: Decide how this logic should operate" << std::endl;
    //ASSERT_EQ(orinSysStatus, ERROR) << "Orin did not enter ERROR state after Nano sent error";
}

TEST_F(SystemIntegrationTest, OrinPrimary_NanoBecomesPrimary){
    nanoSysStatus = PRIMARY; 
    orinSysStatus = PRIMARY;
    
    for (int i=0; i<10; i++) {
        orinLink->spin_once(); orinLink->send_heartbeat();
        nanoLink->spin_once(); nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    uint8_t status = (uint8_t)PRIMARY;
    std::cout << "[TEST] Injecting ID 211 (Change in SystemStatus)" << std::endl;
    nanoLink->send_data(211, &status, sizeof(status));
    
    for (int i=0; i<10; i++) {
        orinLink->spin_once(); orinLink->send_heartbeat();
        nanoLink->spin_once(); nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(orinSysStatus, STANDBY) << "Orin did not enter STANDBY state after Nano attempted to become PRIMARY while Orin was PRIMARY";
    ASSERT_EQ(nanoSysStatus, STOP);
}

TEST_F(SystemIntegrationTest, NanoShutdown){
    nanoSysStatus = STANDBY; 
    orinSysStatus = PRIMARY;
    
    for (int i=0; i<10; i++) {
        orinLink->spin_once(); orinLink->send_heartbeat();
        nanoLink->spin_once(); nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[TEST] Injecting ID 500 (System Shutdown)" << std::endl;
    nanoLink->send_data(500, "", 0);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    orinLink->spin_once();
    ASSERT_EQ(orinSysStatus, SINGLE_FC) << "Orin did not enter SINGLE_FC as expected";
}

TEST_F(SystemIntegrationTest, NanoReboot){
    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;
    
    for (int i=0; i<10; i++) {
        orinLink->spin_once(); orinLink->send_heartbeat();
        nanoLink->spin_once(); nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[TEST] Injecting ID 500 (System Shutdown)" << std::endl;
    nanoLink->send_data(500, "", 0);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    orinLink->spin_once();orinLink->send_heartbeat();

    ASSERT_EQ(orinSysStatus, SINGLE_FC) << "Orin did not enter SINGLE_FC as expected";

    std::cout << "[TEST] Injecting ID 501 (System Shutdown)" << std::endl;
    nanoLink->spin_once(); nanoLink->send_heartbeat();
    nanoLink->send_data(501, "", 0);
    
     for (int i=0; i<10; i++) {
        orinLink->spin_once(); orinLink->send_heartbeat();
        nanoLink->spin_once(); nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
}

TEST_F(SystemIntegrationTest, CanParserTest) {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    uint32_t id = 0;
    id |= (10 & 0x1F) << 24; // DEV_TYPE_FC (10)
    id |= (15 & 0xFF) << 16; // MFR_CUSTOM (15)
    id |= (1  & 0x3F) << 10; // API_CLASS_STATUS (1)
    id |= (1  & 0x0F) << 6;  // API_IDX_HB (1)
    id |= (2  & 0x3F) << 0;  // Sender ID (2 = Nano)
    
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
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    uint32_t id = 0;
    id |= (10 & 0x1F) << 24; // DEV_TYPE_FC
    id |= (15 & 0xFF) << 16; // MFR_CUSTOM
    id |= (2  & 0x3F) << 10; // API_CLASS_CONTROL (2)
    id |= (2  & 0x0F) << 6;  // API_IDX_DATA (2)
    id |= (2  & 0x3F) << 0;  // Sender ID
    frame.can_id = id | CAN_EFF_FLAG;
    frame.can_dlc = sizeof(CanDataPayload);

    CanDataPayload data_load;
    data_load.message_id = ID_JAXIS_MSG;
    
    JoystickAxis joyMsg {0, 0, 1.0f}; // ID 0, Axis 0, Val 1.0
    std::memcpy(data_load.data, &joyMsg, sizeof(joyMsg));
    std::memcpy(frame.data, &data_load, sizeof(data_load));


    while(orinLink->spin_once()); orinLink->send_heartbeat();
    ASSERT_TRUE(orinLink->is_remote_alive());

    CanDataPayload parsed_data;
    ASSERT_TRUE(orinCanLink->parse_data(frame, parsed_data));
    
    orinController->onCanDataReceived(parsed_data);


    std::cout << "[TEST] Killing Ethernet..." << std::endl;
    nano_running = false; 
    if (nano_thread.joinable()) nano_thread.join();

    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    orinLink->spin_once(); // Update liveness check

    ASSERT_FALSE(orinLink->is_remote_alive());

    std::cout << "[TEST] Injecting CAN Joystick Command..." << std::endl;
    orinController->onCanDataReceived(parsed_data);
}

TEST_F(SystemIntegrationTest, NormalStartSequence){
    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;
    
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


// --- Abnormal Startup Sequences ---
/* Tested Configurations:
 * Nano Absent, Orin Absent
 * Nano Delayed, Orin Delayed
 * Partial Heartbeat, No CAN - Nano , Orin
 * Partial Heartbeat, No Eth - Nano, Orin
 * Orin Does Not Take Control
*/
// Peer Absent During Startup - Nano
TEST_F(SystemIntegrationTest, AbnormalStart_NanoAbsent){
    nano_running = false;
    if (nano_thread.joinable()) {
        nano_thread.join();
    }
    orinSysStatus = STANDBY;

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    while(orinLink->spin_once());  orinLink->send_heartbeat();
    orinController->alertSystemBoot();
    orinController->queryControl();

    for (int i=0; i<10; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(orinSysStatus, SINGLE_FC) << "Orin did not enter SINGLE_FC as expected";
}

// Peer Absent During Startup - Orin
TEST_F(SystemIntegrationTest, AbnormalStart_OrinAbsent){
    orin_running = false;
    if (orin_thread.joinable()) {
        orin_thread.join();
    }
    orinSysStatus = STANDBY;
    nanoSysStatus = STANDBY;

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    while(nanoLink->spin_once());  nanoLink->send_heartbeat();
    nanoController->alertSystemBoot();
    nanoController->queryControl();

    for (int i=0; i<10; i++) {
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(nanoSysStatus, SINGLE_FC) << "Nano did not enter SINGLE_FC as expected";
}

// Peer Delayed During Startup - Nano
TEST_F(SystemIntegrationTest, AbnormalStart_NanoDelayed){
    nano_running = false;
    if (nano_thread.joinable()) {
        nano_thread.join();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;
    
    orinLink->spin_once(); orinLink->send_heartbeat();
    orinController->alertSystemBoot();
    orinController->queryControl();

    for (int i=0; i<100; i++) {
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(orinSysStatus, SINGLE_FC) << "Orin did not enter SINGLE_FC as expected";

    nanoLink->spin_once(); nanoLink->send_heartbeat();
    std::cout << "[TEST] Injecting ID 501 (System Booted)" << std::endl;
    nanoController->alertSystemBoot();
    
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

// Peer Delayed During Startup - Orin
TEST_F(SystemIntegrationTest, AbnormalStart_OrinDelayed){
    orin_running = false;
    if (orin_thread.joinable()) {
        orin_thread.join();
    }
    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    nanoLink->spin_once(); nanoLink->send_heartbeat();
    nanoController->alertSystemBoot();
    nanoController->queryControl();

    for (int i=0; i<100; i++) {
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(nanoSysStatus, SINGLE_FC) << "Nano did not enter SINGLE_FC as expected";

    orinLink->spin_once(); orinLink->send_heartbeat();
    std::cout << "[TEST] Injecting ID 501 (System Booted)" << std::endl;
    orinController->alertSystemBoot();
    
    for (int i=0; i<10; i++) {
        while(nanoLink->spin_once()); 
        nanoLink->send_heartbeat();
        
        while(orinLink->spin_once()); 
        orinLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(nanoSysStatus, PRIMARY) << "Nano did not enter PRIMARY as expected";
    ASSERT_EQ(orinSysStatus, STANDBY) << "Orin did not enter STANDBY as expected";
}

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


// --- Motor Node Crashes ---
/* Tested configurations:
 * Orin Primary, Nano Primary
 * Orin SINGLE_FC, Nano SINGLE_FC
*/
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