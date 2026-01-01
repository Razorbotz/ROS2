#include <gtest/gtest.h>
#include "MockDeps.hpp"

#define UNIT_TEST 
#include "AegisController.cpp"

class AegisControllerTest : public ::testing::Test {
protected:
    rclcpp::Node::SharedPtr mockNode;
    HeartbeatLink mockLink;
    std::mutex mockMutex;
    RemoteStatus status;
    bool sendRawData = false;
    SystemStatus sysStatus = PRIMARY;
    std::shared_ptr<AegisController> controller;

    void SetUp() override {
        mockNode = rclcpp::Node::make_shared("test_node");
        controller = std::make_shared<AegisController>(
            mockNode, mockLink, mockMutex, status, sendRawData, sysStatus
        );
    }

    void TearDown() override {
        mockLink.reset();
    }
};

// --- TEST 1: Sending Joystick Data ---
TEST_F(AegisControllerTest, SendsJoystickAxisCorrectly) {
    // Action
    controller->sendJoystickAxis(1, 2, 0.5f);

    // Assertion
    ASSERT_EQ(mockLink.sent_packets.size(), 1);
    EXPECT_EQ(mockLink.sent_packets[0].id, 010); // Check ID (Octal 010 = 8 decimal)
    
    // Verify Payload
    JoystickAxis* payload = (JoystickAxis*)mockLink.sent_packets[0].payload.data();
    EXPECT_EQ(payload->joystick_id, 1);
    EXPECT_EQ(payload->axis_id, 2);
    EXPECT_FLOAT_EQ(payload->value, 0.5f);
}

// --- TEST 2: Receiving Critical Error (Packet 404) ---
TEST_F(AegisControllerTest, HandlesWifiLoss) {
    // Initial State
    status.WIFI_UP = true;
    sendRawData = false;

    // Simulate incoming packet 404 (Empty payload implied by case logic, or specific size?)
    controller->on_packet_received(404, nullptr, 0);

    // Assertion
    EXPECT_FALSE(status.WIFI_UP); // Should set WIFI_UP to false
    EXPECT_TRUE(sendRawData);     // Should set sendRawData to true
}

// --- TEST 3: Receiving System Shutdown (Packet 500) ---
TEST_F(AegisControllerTest, HandlesSystemShutdown) {
    status.UP = true;
    sysStatus = PRIMARY;

    controller->on_packet_received(500, nullptr, 0);

    EXPECT_FALSE(status.UP);
    EXPECT_EQ(sysStatus, SINGLE_FC);
}

// --- TEST 4: Receiving Motor Speed (Packet 001) ---
TEST_F(AegisControllerTest, ParsesMotorSpeed) {
    // Construct a valid payload
    MotorSpeed validMsg;
    validMsg.motor_id = 5;
    validMsg.speed = 0.88f;

    uint8_t buffer[sizeof(MotorSpeed)];
    std::memcpy(buffer, &validMsg, sizeof(MotorSpeed));

    ASSERT_NO_THROW(
        controller->on_packet_received(001, buffer, sizeof(MotorSpeed))
    );
}

// --- TEST 5: Check Connection Guard ---
TEST_F(AegisControllerTest, DoesNotSendWhenDisconnected) {
    mockLink.connected = false;
    
    controller->sendJoystickAxis(0, 0, 1.0f);
    
    EXPECT_EQ(mockLink.sent_packets.size(), 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}