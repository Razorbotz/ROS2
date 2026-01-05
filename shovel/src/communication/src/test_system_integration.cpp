#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <chrono>

#include "MockDeps.hpp"
#include "Heartbeat.hpp"

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
    std::shared_ptr<AegisController> orinController;
    std::mutex orinMutex;
    RemoteStatus orinRemoteStatus;
    bool orinRawData = false;
    SystemStatus orinSysStatus = PRIMARY;

    std::shared_ptr<rclcpp::Node> nanoNode;
    std::unique_ptr<HeartbeatLink> nanoLink;
    std::shared_ptr<AegisNanoController> nanoController;
    std::mutex nanoMutex;
    RemoteStatus nanoRemoteStatus;
    bool nanoRawData = false;
    SystemStatus nanoSysStatus = STANDBY;

    std::atomic<bool> system_running {true};
    std::thread nano_thread;
    std::thread orin_thread;

    void SetUp() override {
        // 1. Setup Orin (Sends to Nano)
        orinNode = rclcpp::Node::make_shared("orin_node");
        orinLink = std::make_unique<HeartbeatLink>(ORIN_PORT, LOCAL_IP, NANO_PORT);
        orinLink->init();
        orinController = std::make_shared<AegisController>(
            orinNode, *orinLink, orinMutex, orinRemoteStatus, orinRawData, orinSysStatus
        );
        using namespace std::placeholders;
        orinLink->set_data_callback(
            std::bind(&AegisController::on_packet_received, orinController, _1, _2, _3)
        );

        // 2. Setup Nano (Sends to Orin)
        nanoNode = rclcpp::Node::make_shared("nano_node");
        nanoLink = std::make_unique<HeartbeatLink>(NANO_PORT, LOCAL_IP, ORIN_PORT);
        nanoLink->init();
        nanoController = std::make_shared<AegisNanoController>(
            nanoNode, *nanoLink, nanoMutex, nanoRemoteStatus, nanoRawData, nanoSysStatus
        );
        nanoLink->set_data_callback(
            std::bind(&AegisNanoController::on_packet_received, nanoController, _1, _2, _3)
        );

        // 3. Start Communication Threads
        orin_thread = std::thread([this]() {
            while (system_running) {
                orinLink->spin_once();
                orinLink->send_heartbeat();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        nano_thread = std::thread([this]() {
            while (system_running) {
                nanoLink->spin_once();
                nanoLink->send_heartbeat();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        // 4. Wait for connection to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        ASSERT_TRUE(orinLink->is_remote_alive()) << "Orin cannot see Nano";
        ASSERT_TRUE(nanoLink->is_remote_alive()) << "Nano cannot see Orin";
    }

    void TearDown() override {
        system_running = false;
        if (orin_thread.joinable()) orin_thread.join();
        if (nano_thread.joinable()) nano_thread.join();
        orinLink->close_socket();
        nanoLink->close_socket();
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

// --- TEST 1: Orin Sends Joystick -> Nano Receives ---
TEST_F(SystemIntegrationTest, OrinControlsNano) {
    // 1. Send data from Orin
    std::cout << "[TEST] Sending Joystick Button Press..." << std::endl;
    orinController->sendJoystickButton(0, 5, 1); // Joystick 0, Button 5, Pressed

    // 2. Wait for transmission
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 3. Verify
    // Since AegisNanoController just prints to cout, we check the console output visually.
    // Ideally, you would add a "last_received_button" member to AegisNanoController
    // to allow this test to ASSERT_EQ(nanoController->last_btn, 5);
}

TEST_F(SystemIntegrationTest, OrinQuerysNanoControl_OrinPrimary) {
    // 1. Send control query from Orin to Nano
    std::cout << "[TEST] Query Control" << std::endl;
    orinController->queryControl();

    for (int i = 0; i < 10; i++) {
        orinLink->spin_once();
        orinLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "orinSysStatus" << (int)orinSysStatus << std::endl;
    ASSERT_TRUE(orinSysStatus == PRIMARY) << "Orin should be PRIMARY";
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
    ASSERT_EQ(orinSysStatus, ERROR) << "Orin did not enter ERROR state after Nano sent error";
}

TEST_F(SystemIntegrationTest, OrinPrimary_NanoBecomesPrimary){
    nanoSysStatus = STANDBY; 
    orinSysStatus = PRIMARY;
    
    for (int i=0; i<10; i++) {
        orinLink->spin_once(); orinLink->send_heartbeat();
        nanoLink->spin_once(); nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    uint8_t status = 0;
    std::cout << "[TEST] Injecting ID 211 (Change in SystemStatus)" << std::endl;
    nanoLink->send_data(211, &status, sizeof(status));
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    orinLink->spin_once();
    ASSERT_EQ(orinSysStatus, ERROR) << "Orin did not enter error state after Nano attempted to become PRIMARY while Orin was PRIMARY";
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

TEST_F(SystemIntegrationTest, NormalStartSequence){
    nanoSysStatus = STANDBY; 
    orinSysStatus = STANDBY;
    
    for (int i=0; i<10; i++) {
        orinLink->spin_once(); orinLink->send_heartbeat();
        nanoLink->spin_once(); nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[TEST] Injecting ID 501 (System Booted)" << std::endl;
    nanoLink->send_data(501, "", 0);
    orinLink->send_data(501, "", 0);
    
    for (int i=0; i<10; i++) {
        orinLink->spin_once(); orinLink->send_heartbeat();
        nanoLink->spin_once(); nanoLink->send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(orinSysStatus, PRIMARY) << "Orin did not enter PRIMARY as expected";
    ASSERT_EQ(nanoSysStatus, STANDBY) << "Nano did not enter STANDBY as expected";
}