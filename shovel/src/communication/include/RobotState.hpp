#pragma once
#include <cstdint>
#include <string>

struct Falcon {
    uint8_t device_id;
    uint16_t voltage;
    uint16_t current;
    float output_percent;
    uint8_t temperature;
    float sensor_position;
    float sensor_velocity;
    float max_current;
    bool temp_disable;
    bool error;
};

struct Kraken {
    uint8_t device_id;
    uint16_t voltage;
    uint16_t current;
    float output_percent;
    uint8_t temperature;
    float sensor_position;
    float sensor_velocity;
    float max_current;
    bool temp_disable;
    bool error;
};

struct Talon {
    uint8_t device_id;
    uint16_t voltage;
    uint16_t current;
    float output_percent;
    uint8_t temperature;
    uint16_t sensor_position;
    float sensor_velocity;
    float max_current;
    bool temp_disable;
};

struct Linear {
    uint8_t motor_number;
    float speed;
    uint16_t potentiometer;
    uint8_t time_without_change;
    uint16_t max;
    uint16_t min;
    std::string error;
    bool at_min;
    bool at_max;
    float distance;
    bool sensorless;
};

struct AutonomyState {
    std::string robot_state;
    std::string excavation_state;
    std::string error_state;
    std::string diagnostics_state;
    std::string tilt_state;
    std::string dump_state;
    std::string bucket_state;
    std::string arms_state;
    float dest_x = 0.0f;
    float dest_z = 0.0f;
};

struct ZedState {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    bool aruco;
};

struct DrivetrainState {
    float f1_vel;
    float f1_rpm;
    float f1_speed;
    float f2_vel;
    float f2_rpm;
    float f2_speed;
    float f3_vel;
    float f3_rpm;
    float f3_speed;
    float f4_vel;
    float f4_rpm;
    float f4_speed;
};

struct SystemState {
    int32_t  rssi;
    std::string wifi;
    std::string can_bus;
    bool using_can1;
    int32_t  rx_packets;
    int32_t  tx_packets;
    std::string can_bus2;
    int32_t  rx_packets2;
    int32_t  tx_packets2;
    int32_t  first_motor;
    int32_t  second_motor;
    int32_t  num_breaks;
};

struct LidarState {
    int16_t distance_mm;
    float distance_m;
};