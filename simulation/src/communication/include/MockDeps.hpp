#pragma once
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <mutex>

// g++ -o run_tests test_aegis_controller.cpp ../src/BinaryMessage.cpp -lgtest -lpthread -std=c++17 -I. -I../include -DUNIT_TEST
// ./run_tests

// --- MOCK ROS2 ONLY ---
namespace rclcpp {
    class Logger {
    public:
        void info(const char* msg) { std::cout << "[INFO] " << msg << std::endl; }
        void warn(const char* msg) { std::cout << "[WARN] " << msg << std::endl; }
        void error(const char* msg) { std::cerr << "[ERROR] " << msg << std::endl; }
    };

    class Node {
    public:
        using SharedPtr = std::shared_ptr<Node>;
        Logger get_logger() { return Logger(); }
        static SharedPtr make_shared(std::string name) { return std::make_shared<Node>(); }
    };
}

// Macros to replace ROS logging macros
#define RCLCPP_INFO(logger, ...) printf("[INFO] "); printf(__VA_ARGS__); printf("\n")
#define RCLCPP_WARN(logger, ...) printf("[WARN] "); printf(__VA_ARGS__); printf("\n")
#define RCLCPP_ERROR(logger, ...) printf("[ERROR] "); printf(__VA_ARGS__); printf("\n")