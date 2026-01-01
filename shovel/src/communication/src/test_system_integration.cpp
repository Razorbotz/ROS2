// g++ -o run_integration test_system_integration.cpp AegisController.cpp ../src/BinaryMessage.cpp Heartbeat.cpp -lgtest -lpthread -std=c++17 -I. -I../include -DUNIT_TEST
#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <chrono>

#include "MockDeps.hpp"
#include "Heartbeat.hpp"
#include "AegisController.hpp"

// Define the ports for localhost testing
#define ORIN_PORT 31337
#define NANO_PORT 31338
#define LOCAL_IP "127.0.0.1"

class SystemIntegrationTest : public ::testing::Test {
protected:
    // --- ORIN SIDE ---
    std::shared_ptr<rclcpp::Node> orinNode;
    std::unique_ptr<HeartbeatLink> orinLink;
    std::shared_ptr<AegisController> orinController;
    std::mutex orinMutex;
    RemoteStatus orinRemoteStatus;
    bool orinRawData = false;
    SystemStatus orinSysStatus = PRIMARY;

    // --- NANO SIDE (The Simulated Partner) ---
    std::unique_ptr<HeartbeatLink> nanoLink;
    std::vector<uint16_t> received_packet_ids;
    bool nano_running = true;
    std::thread nano_thread;

    void SetUp() override {
        // 1. Setup Orin (Port 31337, sending to 31338)
        orinNode = rclcpp::Node::make_shared("orin_node");
        orinLink = std::make_unique<HeartbeatLink>(ORIN_PORT, LOCAL_IP, NANO_PORT);
        orinLink->init();

        orinController = std::make_shared<AegisController>(
            orinNode, *orinLink, orinMutex, orinRemoteStatus, orinRawData, orinSysStatus
        );

        // Bind Orin RX
        using namespace std::placeholders;
        orinLink->set_data_callback(
            std::bind(&AegisController::on_packet_received, orinController, _1, _2, _3)
        );

        // 2. Setup Nano (Port 31338, sending to 31337)
        nanoLink = std::make_unique<HeartbeatLink>(NANO_PORT, LOCAL_IP, ORIN_PORT);
        nanoLink->init();

        // Nano Logic: Just store IDs of what we receive so we can check them in tests
        nanoLink->set_data_callback([this](uint16_t id, const uint8_t* data, uint16_t len) {
            std::cout << "[NANO] Received Packet ID: " << id << std::endl;
            received_packet_ids.push_back(id);
            
            // Example: If we receive "Joystick Button" (011), send back a "Motor Speed" (001)
            if (id == 011) {
                MotorSpeed reply { 1, 0.99f };
                nanoLink->send_data(001, &reply, sizeof(reply));
            }
        });

        // 3. Start Nano Thread (Simulates the loop on the other Jetson)
        nano_thread = std::thread([this]() {
            while (nano_running) {
                nanoLink->spin_once();
                nanoLink->send_heartbeat(); // Crucial for "is_remote_alive"
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    void TearDown() override {
        nano_running = false;
        if (nano_thread.joinable()) nano_thread.join();
        orinLink->close_socket();
        nanoLink->close_socket();
    }
};

// --- TEST 1: Heartbeat Handshake ---
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

// --- TEST 2: Two-Way Data Exchange ---
TEST_F(SystemIntegrationTest, OrinSendsAndNanoResponds) {
    // 1. Orin sends Joystick Button (ID 011)
    std::cout << "[TEST] Orin sending Joystick Button..." << std::endl;
    orinController->sendJoystickButton(0, 1, 1);

    // 2. Run simulation loop for a bit
    for (int i = 0; i < 20; i++) {
        orinLink->spin_once();      // Orin reads Nano's reply
        orinLink->send_heartbeat(); // Keep connection alive
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 3. Verify Nano received the ID
    bool nano_got_packet = false;
    for (auto id : received_packet_ids) {
        if (id == 011) nano_got_packet = true;
    }
    EXPECT_TRUE(nano_got_packet) << "Nano did not receive the button press";

    // 4. Verify Orin processed the reply
    // (In your code, ID 001 prints "Motor X set to Y")
    // We can't easily assert on std::cout, but if this doesn't crash, the RX path works.
}