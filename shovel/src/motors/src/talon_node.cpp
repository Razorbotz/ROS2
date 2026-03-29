#include <string>
#include <iostream>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <messages/msg/key_state.hpp>
#include <messages/msg/talon_status.hpp>

#include "talon_hal.hpp"
#ifdef HAS_PHOENIX5
#include "talon_hardware.hpp"
#endif
#include "talon_sim.hpp"
#include "utils/utils.hpp"


/** @file
 * @brief Unified Talon motor controller node.
 *
 * This node controls a single Talon SRX motor through a hardware
 * abstraction layer (TalonHAL). The same node binary is used for both
 * physical hardware and Gazebo simulation — the `use_sim` parameter
 * selects which backend is active.
 *
 * All ROS topic interfaces, safety logic, heartbeat monitoring,
 * and status publishing are shared code. Only the low-level motor
 * communication differs between the two backends.
 *
 * Subscribed topics:
 *   - {speed_topic} (Float32)    — percent output [-1, 1]
 *   - {position_topic} (Int32)   — position target (potentiometer units)
 *   - {stop_topic} (Bool)        — enable/disable publishing
 *   - STOP (Empty)               — emergency stop
 *   - GO (Empty)                 — enable motors
 *   - comm_heartbeat (Empty)     — communication heartbeat
 *   - logic_heartbeat (Empty)    — logic node heartbeat
 *   - key (KeyState)             — keyboard input
 *
 * Published topics:
 *   - {info_topic} (TalonStatus) — motor status telemetry
 *   - reset_topic (String)       — reset command
 *
 * Parameters:
 *   - use_sim (bool, default false)         — use simulation backend
 *   - motor_number (int, default 1)         — CAN device ID
 *   - can_interface (string, default "can0") — CAN bus name
 *   - diagnostics_port (int, default 1)     — Phoenix diagnostics port
 *   - info_topic (string)                   — status publish topic
 *   - speed_topic (string)                  — speed subscribe topic
 *   - position_topic (string)               — position subscribe topic
 *   - stop_topic (string)                   — publish enable topic
 *   - invert_motor (bool, default false)
 *   - kP, kI, kD, kF (double)              — PID gains
 *   - publishing_delay (int, default 0)     — status publish interval ms
 *   - op_mode (int, default 0)              — thermal protection mode
 *   - print_data (bool, default false)      — verbose logging
 *   - reset_topic (string, default "1")     — reset identifier
 */


// ============================================================================
//  Globals
// ============================================================================

rclcpp::Node::SharedPtr nodeHandle;
std::unique_ptr<TalonHAL> hal;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> resetPublisher;

bool GO = false;
bool TEMP_DISABLE = false;
bool publish = false;
bool usePosition = false;
bool printData = false;

float currentSpeed = 0.0;
int currentPosition = 0;
int op_mode = 0;
float maxCurrent = 0.0;

std::string resetString = "";

std::chrono::time_point<std::chrono::high_resolution_clock> commPrevious;
std::chrono::time_point<std::chrono::high_resolution_clock> logicPrevious;


// ============================================================================
//  Callbacks — identical for both hardware and simulation
// ============================================================================

void stopCallback(std_msgs::msg::Empty::SharedPtr) {
    if (printData)
        RCLCPP_INFO(nodeHandle->get_logger(), "STOP");
    GO = false;
    hal->setPercentOutput(0.0);
    currentSpeed = 0.0;
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
        if (usePosition) {
            hal->setPosition(currentPosition);
        } else {
            hal->setPercentOutput(currentSpeed);
        }
    }
}

void speedCallback(const std_msgs::msg::Float32::SharedPtr speed) {
    if (printData)
        RCLCPP_INFO(nodeHandle->get_logger(), "Speed: %f", speed->data);

    if (publish)
        hal->setPercentOutput(speed->data);

    currentSpeed = speed->data;
    usePosition = false;
}

void positionCallback(const std_msgs::msg::Int32::SharedPtr position) {
    if (printData)
        RCLCPP_INFO(nodeHandle->get_logger(), "Position: %d", position->data);

    if (publish)
        hal->setPosition(position->data);

    currentPosition = position->data;
    usePosition = true;
}

void checkTemperature(double temperature) {
    switch (op_mode) {
        case 0: TEMP_DISABLE = (temperature > 70); break;
        case 1: TEMP_DISABLE = (temperature > 80); break;
        case 2: TEMP_DISABLE = (temperature > 90); break;
    }
}

void keyCallback(const messages::msg::KeyState::SharedPtr keyState) {
    if (printData)
        std::cout << "Key " << keyState->key << " " << keyState->state << std::endl;

    if (keyState->key == 98 && keyState->state == 1) {
        std_msgs::msg::String reset;
        reset.data = resetString;
        resetPublisher->publish(reset);
    }
}


// ============================================================================
//  Main
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    nodeHandle = rclcpp::Node::make_shared("talon");

    RCLCPP_INFO(nodeHandle->get_logger(), "Starting unified talon node");

    // --- Parameters ---
    bool useSim        = ::utils::getParameter<bool>(nodeHandle, "use_sim", false);
    int motorNumber    = ::utils::getParameter<int>(nodeHandle, "motor_number", 1);
    int diagnosticsPort = ::utils::getParameter<int>(nodeHandle, "diagnostics_port", 1);
    std::string canInterface = ::utils::getParameter<std::string>(nodeHandle, "can_interface", "can0");

    std::string infoTopic     = ::utils::getParameter<std::string>(nodeHandle, "info_topic", "unset");
    std::string speedTopic    = ::utils::getParameter<std::string>(nodeHandle, "speed_topic", "unset");
    std::string positionTopic = ::utils::getParameter<std::string>(nodeHandle, "position_topic", "unset");
    std::string stopTopic     = ::utils::getParameter<std::string>(nodeHandle, "stop_topic", "unset");
    resetString = ::utils::getParameter<std::string>(nodeHandle, "reset_topic", "1");

    bool invertMotor = ::utils::getParameter<bool>(nodeHandle, "invert_motor", false);
    double kP = ::utils::getParameter<double>(nodeHandle, "kP", 1.0);
    double kI = ::utils::getParameter<double>(nodeHandle, "kI", 0.0);
    double kD = ::utils::getParameter<double>(nodeHandle, "kD", 0.0);
    double kF = ::utils::getParameter<double>(nodeHandle, "kF", 0.0);
    int publishingDelay = ::utils::getParameter<int>(nodeHandle, "publishing_delay", 0);
    op_mode    = ::utils::getParameter<int>(nodeHandle, "op_mode", 0);
    printData  = ::utils::getParameter<bool>(nodeHandle, "print_data", false);

    // --- Create the appropriate HAL backend ---
    if (useSim) {
        RCLCPP_INFO(nodeHandle->get_logger(), "Using SIMULATION backend");
        hal = std::make_unique<TalonSim>();
    } else {
#ifdef HAS_PHOENIX5
        RCLCPP_INFO(nodeHandle->get_logger(), "Using HARDWARE backend");
        hal = std::make_unique<TalonHardware>();
#else
        RCLCPP_FATAL(nodeHandle->get_logger(),
                     "Hardware backend requested but Phoenix 5 was not available at build time. "
                     "Set use_sim:=true for simulation.");
        return 1;
#endif
    }

    if (!hal->initialize(nodeHandle, motorNumber, canInterface, diagnosticsPort,
                         invertMotor, kP, kI, kD, kF)) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to initialize TalonHAL");
        return 1;
    }

    // --- ROS interface (shared between both backends) ---
    auto talonStatusPublisher = nodeHandle->create_publisher<messages::msg::TalonStatus>(infoTopic, 1);
    resetPublisher = nodeHandle->create_publisher<std_msgs::msg::String>("reset_topic", 1);

    auto speedSubscriber    = nodeHandle->create_subscription<std_msgs::msg::Float32>(speedTopic, 1, speedCallback);
    auto positionSubscriber = nodeHandle->create_subscription<std_msgs::msg::Int32>(positionTopic, 1, positionCallback);
    auto stopSubscriber     = nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP", 1, stopCallback);
    auto goSubscriber       = nodeHandle->create_subscription<std_msgs::msg::Empty>("GO", 1, goCallback);
    auto commHeartbeatSub   = nodeHandle->create_subscription<std_msgs::msg::Empty>("comm_heartbeat", 1, commHeartbeatCallback);
    auto logicHeartbeatSub  = nodeHandle->create_subscription<std_msgs::msg::Empty>("logic_heartbeat", 1, logicHeartbeatCallback);
    auto keySubscriber      = nodeHandle->create_subscription<messages::msg::KeyState>("key", 1, keyCallback);
    auto publishSubscriber  = nodeHandle->create_subscription<std_msgs::msg::Bool>(stopTopic, 1, publishCallback);

    RCLCPP_INFO(nodeHandle->get_logger(), "Subscribers configured");

    // --- Main loop ---
    rclcpp::Rate rate(100);
    auto start = std::chrono::high_resolution_clock::now();
    commPrevious = std::chrono::high_resolution_clock::now();
    logicPrevious = std::chrono::high_resolution_clock::now();

    messages::msg::TalonStatus talonStatus;

    while (rclcpp::ok()) {
        if (GO && publish)
            hal->feedEnable(100);

        auto finish = std::chrono::high_resolution_clock::now();

        // --- Publish status ---
        if (std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() > publishingDelay) {
            talonStatus.device_id          = hal->getDeviceID();
            talonStatus.bus_voltage        = hal->getBusVoltage();
            talonStatus.output_current     = hal->getOutputCurrent();
            talonStatus.output_voltage     = hal->getMotorOutputVoltage();
            talonStatus.output_percent     = hal->getMotorOutputPercent();
            talonStatus.temperature        = hal->getTemperature();
            talonStatus.sensor_position    = hal->getSensorPosition();
            talonStatus.sensor_velocity    = hal->getSensorVelocity();
            talonStatus.closed_loop_error  = hal->getClosedLoopError();
            talonStatus.integral_accumulator = hal->getIntegralAccumulator();
            talonStatus.error_derivative   = hal->getErrorDerivative();
            talonStatus.temp_disable       = TEMP_DISABLE;

            double outputCurrent = hal->getOutputCurrent();
            if (outputCurrent > maxCurrent) {
                maxCurrent = outputCurrent;
            }
            talonStatus.max_current = maxCurrent;

            talonStatusPublisher->publish(talonStatus);
            checkTemperature(hal->getTemperature());
            start = std::chrono::high_resolution_clock::now();
        }

        // --- Safety: disable on heartbeat timeout or overtemp ---
        if (std::chrono::duration_cast<std::chrono::milliseconds>(finish - commPrevious).count() > 100 ||
            TEMP_DISABLE ||
            std::chrono::duration_cast<std::chrono::milliseconds>(finish - logicPrevious).count() > 100)
        {
            if (TEMP_DISABLE && printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "Temp Disable");
            if (std::chrono::duration_cast<std::chrono::milliseconds>(finish - commPrevious).count() > 100 && printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "comm disable");
            if (std::chrono::duration_cast<std::chrono::milliseconds>(finish - logicPrevious).count() > 100 && printData)
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