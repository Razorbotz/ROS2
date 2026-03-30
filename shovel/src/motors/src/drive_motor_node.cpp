#include <string>
#include <iostream>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <messages/msg/key_state.hpp>

#include "drive_motor_hal.hpp"
#ifdef HAS_PHOENIX5
#include "falcon_hardware.hpp"
#endif
#ifdef HAS_PHOENIX6
#include "kraken_hardware.hpp"
#endif
#include "drive_motor_sim.hpp"
#include "utils/utils.hpp"


/** @file
 * @brief Unified drive motor node for Falcon 500, Kraken x60, and simulation.
 *
 * This node controls a single drive motor through the DriveMotorHAL
 * abstraction. The `motor_type` parameter selects the backend:
 *   - "falcon"  -> Phoenix 5 TalonFX (Falcon 500)
 *   - "kraken"  -> Phoenix 6 TalonFX (Kraken x60)
 *   - "sim"     -> Gazebo simulation bridge
 *
 * All ROS topic handling, safety logic, heartbeat monitoring,
 * overcurrent detection, and status publishing are shared code.
 *
 * Publishes FalconStatus or KrakenStatus to maintain backward
 * compatibility with the autonomy node and client GUI. The drivetrain
 * node uses the motor_type to know which unit conversions to apply.
 *
 * Subscribed topics:
 *   - {speed_topic} (Float32)    — percent output [-1, 1]
 *   - {user_topic} (Float32)     — user/manual speed
 *   - {stop_topic} (Bool)        — enable/disable publishing
 *   - STOP (Empty)               — emergency stop
 *   - GO (Empty)                 — enable motors
 *   - comm_heartbeat (Empty)     — communication heartbeat
 *   - logic_heartbeat (Empty)    — logic node heartbeat
 *   - key (KeyState)             — keyboard input
 *
 * Published topics:
 *   - {info_topic} (FalconStatus) — motor status telemetry
 *
 * Parameters:
 *   - motor_type (string, default "falcon")  — "falcon", "kraken", or "sim"
 *   - motor_number (int)
 *   - can_interface (string, default "can0")
 *   - diagnostics_port (int)
 *   - info_topic, speed_topic, user_topic, stop_topic (string)
 *   - invert_motor (bool)
 *   - kP, kI, kD, kF (double)
 *   - supply_current_limit (double, default 70.0)
 *   - publishing_delay (int, default 0)
 *   - op_mode (int, default 0)
 *   - print_data (bool, default false)
 */


// ============================================================================
//  Globals
// ============================================================================

rclcpp::Node::SharedPtr nodeHandle;
std::unique_ptr<DriveMotorHAL> hal;

bool GO = false;
bool TEMP_DISABLE = false;
bool publish = false;
bool printData = false;
bool error = false;
bool restarted = false;
bool resetSent = false;

float Speed = 0.0;
float maxCurrent = 0.0;
int opMode = 0;

int resetCooldownMs = 2000;

std::chrono::time_point<std::chrono::high_resolution_clock> commPrevious;
std::chrono::time_point<std::chrono::high_resolution_clock> logicPrevious;
std::chrono::time_point<std::chrono::high_resolution_clock> lastResetTime;


// ============================================================================
//  Callbacks
// ============================================================================

void stopCallback(std_msgs::msg::Empty::SharedPtr) {
    if (printData)
        RCLCPP_INFO(nodeHandle->get_logger(), "STOP");
    GO = false;
    if (publish) hal->setPercentOutput(0.0);
    Speed = 0.0;
}

void goCallback(std_msgs::msg::Empty::SharedPtr) {
    if (printData)
        RCLCPP_INFO(nodeHandle->get_logger(), "GO");
    GO = true;
}

void commHeartbeatCallback(std_msgs::msg::Empty::SharedPtr) {
    commPrevious = std::chrono::high_resolution_clock::now();
}

void logicHeartbeatCallback(std_msgs::msg::Empty::SharedPtr) {
    logicPrevious = std::chrono::high_resolution_clock::now();
}

void publishCallback(std_msgs::msg::Bool::SharedPtr pub) {
    publish = pub->data;
    if (publish) {
        hal->setPercentOutput(Speed);
    }
}

void speedCallback(const std_msgs::msg::Float32::SharedPtr speed) {
    if (printData)
        RCLCPP_INFO(nodeHandle->get_logger(), "Speed: %f", speed->data);
    if (speed->data != Speed) {
        if (publish) hal->setPercentOutput(speed->data);
        Speed = speed->data;
    }
}

void userSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed) {
    if (printData)
        RCLCPP_INFO(nodeHandle->get_logger(), "User speed: %f", speed->data);
    if (speed->data != Speed) {
        if (publish) hal->setPercentOutput(speed->data);
        Speed = speed->data;
    }
}

void checkTemperature(double temperature) {
    switch (opMode) {
        case 0: TEMP_DISABLE = (temperature > 70); break;
        case 1: TEMP_DISABLE = (temperature > 80); break;
        case 2: TEMP_DISABLE = (temperature > 90); break;
    }
}

void keyCallback(const messages::msg::KeyState::SharedPtr keyState) {
    if (printData)
        std::cout << "Key " << keyState->key << " " << keyState->state << std::endl;
}


// ============================================================================
//  Main
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    nodeHandle = rclcpp::Node::make_shared("drive_motor");

    RCLCPP_INFO(nodeHandle->get_logger(), "Starting unified drive motor node");

    // --- Parameters ---
    std::string motorType = ::utils::getParameter<std::string>(nodeHandle, "motor_type", "falcon");
    int motorNumber       = ::utils::getParameter<int>(nodeHandle, "motor_number", 1);
    int diagnosticsPort = utils::getParameter<int>(nodeHandle, "diagnostics_port", 1250);
    std::string canInterface = ::utils::getParameter<std::string>(nodeHandle, "can_interface", "can0");

    std::string infoTopic  = ::utils::getParameter<std::string>(nodeHandle, "info_topic", "unset");
    std::string speedTopic = ::utils::getParameter<std::string>(nodeHandle, "speed_topic", "unset");
    std::string userTopic  = ::utils::getParameter<std::string>(nodeHandle, "user_topic", "unset");
    std::string stopTopic  = ::utils::getParameter<std::string>(nodeHandle, "stop_topic", "unset");

    bool invertMotor = ::utils::getParameter<bool>(nodeHandle, "invert_motor", false);
    double kP = ::utils::getParameter<double>(nodeHandle, "kP", 1.0);
    double kI = ::utils::getParameter<double>(nodeHandle, "kI", 0.0);
    double kD = ::utils::getParameter<double>(nodeHandle, "kD", 0.0);
    double kF = ::utils::getParameter<double>(nodeHandle, "kF", 0.0);
    double supplyCurrentLimit = ::utils::getParameter<double>(nodeHandle, "supply_current_limit", 70.0);
    int publishingDelay = ::utils::getParameter<int>(nodeHandle, "publishing_delay", 0);
    opMode    = ::utils::getParameter<int>(nodeHandle, "op_mode", 0);
    printData = ::utils::getParameter<bool>(nodeHandle, "print_data", false);

    if (motorType == "sim") {
        publish = true; // Auto-enable motors in simulation
    }

    // In DriveMotorSim constructor, before anything else:
    // --- Create the appropriate HAL backend ---
    if (motorType == "kraken") {
#ifdef HAS_PHOENIX6
        RCLCPP_INFO(nodeHandle->get_logger(), "Using KRAKEN x60 (Phoenix 6) backend");
        hal = std::make_unique<KrakenHardware>();
#else
        RCLCPP_FATAL(nodeHandle->get_logger(),
                     "Kraken backend requested but Phoenix 6 was not available at build time");
        return 1;
#endif
    } else if (motorType == "sim") {
        RCLCPP_INFO(nodeHandle->get_logger(), "Using SIMULATION backend");
        hal = std::make_unique<DriveMotorSim>();
    } else if (motorType == "falcon") {
#ifdef HAS_PHOENIX5
        RCLCPP_INFO(nodeHandle->get_logger(), "Using FALCON 500 (Phoenix 5) backend");
        hal = std::make_unique<FalconHardware>();
#else
        RCLCPP_FATAL(nodeHandle->get_logger(),
                     "Falcon backend requested but Phoenix 5 was not available at build time");
        return 1;
#endif
    } else {
        RCLCPP_FATAL(nodeHandle->get_logger(),
                     "Unknown motor_type '%s'. Use 'falcon', 'kraken', or 'sim'",
                     motorType.c_str());
        return 1;
    }

    if (!hal->initialize(nodeHandle, motorNumber, canInterface, diagnosticsPort,
                         invertMotor, kP, kI, kD, kF, supplyCurrentLimit)) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to initialize drive motor HAL");
        return 1;
    }

    // --- ROS interface (shared across all backends) ---
    // Status publisher is created by the HAL (FalconStatus or KrakenStatus)
    hal->createStatusPublisher(infoTopic);

    auto speedSubscriber     = nodeHandle->create_subscription<std_msgs::msg::Float32>(speedTopic, 1, speedCallback);
    auto userSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>(userTopic, 1, userSpeedCallback);
    auto stopSubscriber      = nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP", 1, stopCallback);
    auto goSubscriber        = nodeHandle->create_subscription<std_msgs::msg::Empty>("GO", 1, goCallback);
    auto commHeartbeatSub    = nodeHandle->create_subscription<std_msgs::msg::Empty>("comm_heartbeat", 1, commHeartbeatCallback);
    auto logicHeartbeatSub   = nodeHandle->create_subscription<std_msgs::msg::Empty>("logic_heartbeat", 1, logicHeartbeatCallback);
    auto keySubscriber       = nodeHandle->create_subscription<messages::msg::KeyState>("key", 1, keyCallback);
    auto publishSubscriber   = nodeHandle->create_subscription<std_msgs::msg::Bool>(stopTopic, 1, publishCallback);

    RCLCPP_INFO(nodeHandle->get_logger(), "Subscribers configured (motor ID %d, type: %s)",
                motorNumber, motorType.c_str());

    // --- Main loop ---
    rclcpp::Rate rate(50);
    auto start = std::chrono::high_resolution_clock::now();
    commPrevious = std::chrono::high_resolution_clock::now();
    logicPrevious = std::chrono::high_resolution_clock::now();
    lastResetTime = std::chrono::high_resolution_clock::now();

    while (rclcpp::ok()) {
        if (GO)hal->feedEnable(100);

        auto finish = std::chrono::high_resolution_clock::now();

        // --- Overcurrent detection and reset (all backends) ---
        if (hal->isOvercurrentTripped()) {
            auto msSinceReset = std::chrono::duration_cast<std::chrono::milliseconds>(
                finish - lastResetTime).count();

            if (!error) {
                RCLCPP_WARN(nodeHandle->get_logger(),
                            "Motor %d: overcurrent fault detected!", hal->getDeviceID());
                error = true;
            }

            if (!resetSent || msSinceReset > resetCooldownMs) {
                RCLCPP_INFO(nodeHandle->get_logger(),
                            "Motor %d: publishing reset (cooldown %d ms)",
                            hal->getDeviceID(), resetCooldownMs);
                lastResetTime = std::chrono::high_resolution_clock::now();
                resetSent = true;
            }
        } else {
            if (error) {
                RCLCPP_INFO(nodeHandle->get_logger(),
                            "Motor %d: fault cleared", hal->getDeviceID());
                hal->clearStickyFaults();
                error = false;
                resetSent = false;
                restarted = true;
            }
        }

        // --- Publish status ---
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                finish - start).count() > publishingDelay)
        {
            double outputCurrent = hal->getOutputCurrent();
            if (outputCurrent > maxCurrent) {
                maxCurrent = outputCurrent;
            }

            hal->publishStatus(TEMP_DISABLE, error, restarted, maxCurrent);
            checkTemperature(hal->getTemperature());
            start = std::chrono::high_resolution_clock::now();
        }

        // --- Safety: disable on heartbeat timeout or overtemp ---
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                finish - commPrevious).count() > 100 ||
            TEMP_DISABLE ||
            std::chrono::duration_cast<std::chrono::milliseconds>(
                finish - logicPrevious).count() > 100)
        {
            if (TEMP_DISABLE && printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "Temp Disable");
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    finish - commPrevious).count() > 100 && printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "comm disable");
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    finish - logicPrevious).count() > 100 && printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "logic disable");

            hal->setPercentOutput(0.0);
            GO = false;
        }

        rate.sleep();
        rclcpp::spin_some(nodeHandle);
    }

    rclcpp::shutdown();
    return 0;
}