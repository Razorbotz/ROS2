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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <cstdint>
#include <zlib.h>

#include <messages/msg/power.hpp>
#include <messages/msg/key_state.hpp>
#include <messages/msg/hat_state.hpp>
#include <messages/msg/button_state.hpp>
#include <messages/msg/axis_state.hpp>
#include <messages/msg/talon_status.hpp>
#include <messages/msg/zed_position.hpp>
#include <messages/msg/linear_status.hpp>
#include <messages/msg/autonomy_status.hpp>
#include <messages/msg/falcon_status.hpp>
#include <messages/msg/kraken_status.hpp>
#include <messages/msg/system_status.hpp>
#include <messages/msg/drivetrain_status.hpp>
#include <messages/msg/lidar_distance.hpp>

#include <BinaryMessage.hpp>
#include <RobotState.hpp>
#include <NetworkUtils.hpp>
#include <MessageUtils.hpp>
#include "utils/utils.hpp"

#include <iostream>
#include <cstring>
#include <net/if.h>
#include <netdb.h>

#define PORT 31337

rclcpp::Node::SharedPtr nodeHandle;

std::string robotName = "unnamed";
std::string interfaceName = "wlP1p1s0";
bool broadcast = true;
bool silentRunning = true;
bool debug = false;

struct sockaddr_in address;
socklen_t addrlen = sizeof(address);
std_msgs::msg::Empty empty;
int new_socket;
std_msgs::msg::Empty heartbeat;

int rssi = 0;

std::atomic<uint64_t> last_client_tx_time_ms{0};

#define LOWER_THRESH 67
#define UPPER_THRESH 80
#define CRIT_THRESH 90

Falcon falcon1, falcon2, falcon3, falcon4;
Kraken kraken1, kraken2, kraken3, kraken4;
Talon talon1, talon2, talon3, talon4;
Linear linear1, linear2, linear3, linear4;
AutonomyState autonomyState;
ZedState zedState;
DrivetrainState drivetrainState;
SystemState systemState;
LidarState lidarState;

// Per-motor "force send booleans on next packet" flags. Set by forceDataResync()
// and cleared by the corresponding send() function once it has emitted them.
// Booleans can't use out-of-band sentinels like the numeric fields do, so a
// dedicated force-flag is the only way to guarantee a resync re-sends them.
struct ForceBoolFlags {
    bool temp_disable = false;
    bool error = false;
};
ForceBoolFlags falcon_force[4];
ForceBoolFlags kraken_force[4];
ForceBoolFlags talon_force[4];   // Talon has no `error` field; only temp_disable is used.

// Helper: behaves like update_if_changed for booleans, but if `force` is true,
// always emits the field and updates the cache, then clears the force flag.
inline void update_bool_with_force(BinaryMessage& msg, bool& changed, bool& old_val,
                                   bool new_val, bool& force, Field_Strings field) {
    if (force || old_val != new_val) {
        changed = true;
        msg.addElementBoolean(field, new_val);
        old_val = new_val;
        force = false;
    }
}

// Resolve the per-motor force-flags struct from a reference to the cache global.
// The send() functions only know the cache by reference, so we map via address.
inline ForceBoolFlags& getForceFlags(Falcon& f) {
    if (&f == &falcon1) return falcon_force[0];
    if (&f == &falcon2) return falcon_force[1];
    if (&f == &falcon3) return falcon_force[2];
    return falcon_force[3]; // falcon4
}
inline ForceBoolFlags& getForceFlags(Kraken& k) {
    if (&k == &kraken1) return kraken_force[0];
    if (&k == &kraken2) return kraken_force[1];
    if (&k == &kraken3) return kraken_force[2];
    return kraken_force[3]; // kraken4
}
inline ForceBoolFlags& getForceFlags(Talon& t) {
    if (&t == &talon1) return talon_force[0];
    if (&t == &talon2) return talon_force[1];
    if (&t == &talon3) return talon_force[2];
    return talon_force[3]; // talon4
}

float current_lidar_dist = -1.0f;
float voltage = 0.0f;
float temperature_val = 0.0f;
std::array<float, 16> currents{};

uint64_t get_time_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}


void forceDataResync() {
    auto resetFalcon = [](Falcon& f) {
        f.device_id = 255; f.voltage = 0xFFFF; f.current = 0xFFFF;
        f.output_percent = -999.0f; f.temperature = 255;
        f.sensor_position = -999999.0f; f.sensor_velocity = -999999.0f;
        f.max_current = -1.0f;
        // Booleans: don't toggle (unreliable — the toggle may match the real value
        // and update_if_changed will then skip the field). Instead, request a
        // forced send on the next status packet via the ForceBoolFlags struct below.
    };
    auto resetTalon = [](Talon& t) {
        t.device_id = 255; t.voltage = 0xFFFF; t.current = 0xFFFF;
        t.output_percent = -999.0f; t.temperature = 255;
        t.sensor_position = -999999.0f; t.sensor_velocity = -999999.0f;
        t.max_current = -1.0f;
    };
    auto resetLinear = [](Linear& l) {
        l.motor_number = 255; l.speed = -999.0f; l.potentiometer = 0xFFFF;
        l.time_without_change = 255; l.max = 0xFFFF; l.min = 0xFFFF;
        l.error = "FORCE_RESYNC"; l.distance = -999.0f;
    };

    resetFalcon(falcon1); resetFalcon(falcon2); resetFalcon(falcon3); resetFalcon(falcon4);
    auto resetKraken = [](Kraken& k) {
        k.device_id = 255; k.voltage = 0xFFFF; k.current = 0xFFFF;
        k.output_percent = -999.0f; k.temperature = 255;
        k.sensor_position = -999999.0f; k.sensor_velocity = -999999.0f;
        k.max_current = -1.0f;
    };
    resetKraken(kraken1); resetKraken(kraken2); resetKraken(kraken3); resetKraken(kraken4);
    resetTalon(talon1); resetTalon(talon2); resetTalon(talon3); resetTalon(talon4);
    resetLinear(linear1); resetLinear(linear2); resetLinear(linear3); resetLinear(linear4);

    // Request a forced re-broadcast of every motor's boolean fields on the next
    // status packet. Each send() function will clear its flags after emitting.
    for (int i = 0; i < 4; ++i) {
        falcon_force[i].temp_disable = true;
        falcon_force[i].error        = true;
        kraken_force[i].temp_disable = true;
        kraken_force[i].error        = true;
        talon_force[i].temp_disable  = true;
    }

    autonomyState.robot_state = "RESYNC"; autonomyState.excavation_state = "RESYNC";
    autonomyState.error_state = "RESYNC"; autonomyState.diagnostics_state = "RESYNC";
    autonomyState.tilt_state = "RESYNC"; autonomyState.dump_state = "RESYNC";
    autonomyState.bucket_state = "RESYNC"; autonomyState.arms_state = "RESYNC";
    autonomyState.dest_x = -99999.0f; autonomyState.dest_z = -99999.0f;

    zedState.x = -99999.0f; zedState.y = -99999.0f; zedState.z = -99999.0f;
    zedState.roll = -999.0f; zedState.pitch = -999.0f; zedState.yaw = -999.0f;
    zedState.aruco = !zedState.aruco;

    drivetrainState.f1_vel = -99999.0f; drivetrainState.f1_rpm = -99999.0f; drivetrainState.f1_speed = -99999.0f;
    drivetrainState.f2_vel = -99999.0f; drivetrainState.f2_rpm = -99999.0f; drivetrainState.f2_speed = -99999.0f;
    drivetrainState.f3_vel = -99999.0f; drivetrainState.f3_rpm = -99999.0f; drivetrainState.f3_speed = -99999.0f;
    drivetrainState.f4_vel = -99999.0f; drivetrainState.f4_rpm = -99999.0f; drivetrainState.f4_speed = -99999.0f;

    systemState.rssi = -1; systemState.wifi = "RESYNC";
    systemState.can_bus = "RESYNC"; systemState.rx_packets = -1;

    voltage = -1.0f; temperature_val = -1.0f; currents.fill(-1.0f);
}

void send(BinaryMessage message) {
    if (!is_sender.load()) return;

    if (debug)
        RCLCPP_INFO(nodeHandle->get_logger(), "Sending message");

    std::shared_ptr<std::list<uint8_t>> byteList = message.getBytes();
    checksum_encode(byteList);

    std::vector<uint8_t> uncompressed_bytes(byteList->begin(), byteList->end());
    uLong uncompressed_size = uncompressed_bytes.size();
    if (uncompressed_size == 0) return;

    uLong compressed_buffer_size = compressBound(uncompressed_size);
    std::vector<uint8_t> compressed_bytes(compressed_buffer_size);

    int compression_result = compress2(
        compressed_bytes.data(), &compressed_buffer_size,
        uncompressed_bytes.data(), uncompressed_size, Z_DEFAULT_COMPRESSION);

    std::vector<uint8_t> payload;

    if (compression_result == Z_OK && compressed_buffer_size < uncompressed_size) {
        payload.reserve(1 + 4 + compressed_buffer_size);
        payload.push_back(1);
        payload.push_back((uncompressed_size >> 24) & 0xFF);
        payload.push_back((uncompressed_size >> 16) & 0xFF);
        payload.push_back((uncompressed_size >> 8) & 0xFF);
        payload.push_back(uncompressed_size & 0xFF);
        payload.insert(payload.end(), compressed_bytes.begin(),
                       compressed_bytes.begin() + compressed_buffer_size);
    } else {
        payload.reserve(1 + uncompressed_size);
        payload.push_back(0);
        payload.insert(payload.end(), uncompressed_bytes.begin(), uncompressed_bytes.end());
    }

    try {
        if (payload.empty()) return;
        sendto(new_socket, payload.data(), payload.size(), 0,
               (struct sockaddr*)&address, addrlen);
        last_client_tx_time_ms.store(get_time_ms(), std::memory_order_relaxed);
    } catch (...) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "ERROR: Exception sending data to client");
    }
}

static void sendClientHeartbeat() {
    if (new_socket <= 0) return;
    if (broadcast) return;
    const uint8_t hb[1] = {0};
    sendto(new_socket, hb, sizeof(hb), 0, (struct sockaddr*)&address, addrlen);
    last_client_tx_time_ms.store(get_time_ms(), std::memory_order_relaxed);
}

void send(std::string messageLabel, const messages::msg::FalconStatus::SharedPtr falconStatus, Falcon& falcon) {
    if (silentRunning) return;
    bool message_changed = false;
    BinaryMessage message(messageLabel);

    uint16_t new_voltage = falconStatus->bus_voltage * 100.0;
    uint16_t new_current = falconStatus->output_current * 100.0;
    uint8_t new_temperature = (uint8_t)falconStatus->temperature;
    uint8_t new_device_id = (uint8_t)falconStatus->device_id;

    update_if_changed(message, message_changed, falcon.device_id,      new_device_id,                 Field_Strings::DeviceID);
    update_if_changed(message, message_changed, falcon.voltage,         new_voltage,                   Field_Strings::BusVoltage);
    update_if_changed(message, message_changed, falcon.current,         new_current,                   Field_Strings::OutputCurrent);
    update_if_changed(message, message_changed, falcon.output_percent,  falconStatus->output_percent,  Field_Strings::OutputPercent);
    update_if_changed(message, message_changed, falcon.temperature,     new_temperature,               Field_Strings::Temperature);
    update_if_changed(message, message_changed, falcon.sensor_position, falconStatus->sensor_position, Field_Strings::SensorPosition);
    update_if_changed(message, message_changed, falcon.sensor_velocity, falconStatus->sensor_velocity, Field_Strings::SensorVelocity);
    update_if_changed(message, message_changed, falcon.max_current,     falconStatus->max_current,     Field_Strings::MaxCurrent);
    ForceBoolFlags& ff = getForceFlags(falcon);
    update_bool_with_force(message, message_changed, falcon.temp_disable, falconStatus->temp_disable, ff.temp_disable, Field_Strings::TempDisable);
    update_bool_with_force(message, message_changed, falcon.error,        falconStatus->error,        ff.error,        Field_Strings::Error);

    if (message_changed) send(message);
}

void send(std::string messageLabel, const messages::msg::KrakenStatus::SharedPtr krakenStatus, Kraken& kraken) {
    if (silentRunning) return;
    bool message_changed = false;
    BinaryMessage message(messageLabel);

    uint16_t new_voltage = krakenStatus->bus_voltage * 100.0;
    uint16_t new_current = krakenStatus->output_current * 100.0;
    uint8_t new_temperature = (uint8_t)krakenStatus->temperature;
    uint8_t new_device_id = (uint8_t)krakenStatus->device_id;

    update_if_changed(message, message_changed, kraken.device_id,       new_device_id,                      Field_Strings::DeviceID);
    update_if_changed(message, message_changed, kraken.voltage,          new_voltage,                        Field_Strings::BusVoltage);
    update_if_changed(message, message_changed, kraken.current,          new_current,                        Field_Strings::OutputCurrent);
    update_if_changed(message, message_changed, kraken.output_percent,   (float)krakenStatus->output_percent, Field_Strings::OutputPercent);
    update_if_changed(message, message_changed, kraken.temperature,      new_temperature,                    Field_Strings::Temperature);
    update_if_changed(message, message_changed, kraken.sensor_position,  (float)krakenStatus->sensor_position, Field_Strings::SensorPosition);
    update_if_changed(message, message_changed, kraken.sensor_velocity,  (float)krakenStatus->sensor_velocity, Field_Strings::SensorVelocity);
    update_if_changed(message, message_changed, kraken.max_current,      (float)krakenStatus->max_current,   Field_Strings::MaxCurrent);
    ForceBoolFlags& kf = getForceFlags(kraken);
    update_bool_with_force(message, message_changed, kraken.temp_disable, krakenStatus->temp_disable, kf.temp_disable, Field_Strings::TempDisable);
    update_bool_with_force(message, message_changed, kraken.error,        krakenStatus->error,        kf.error,        Field_Strings::Error);

    if (message_changed) send(message);
}

void sendKrakenCrit(std::string messageLabel, const messages::msg::KrakenStatus::SharedPtr krakenStatus, Kraken& kraken) {
    if (silentRunning) return;
    if ((float)krakenStatus->output_percent == kraken.output_percent) return;
    BinaryMessage message(messageLabel);
    message.addElementFloat32("Output Percent", krakenStatus->output_percent);
    kraken.output_percent = krakenStatus->output_percent;  // keep cache in sync with what we just sent
    send(message);
}

void send(std::string messageLabel, const messages::msg::TalonStatus::SharedPtr talonStatus, Talon& talon) {
    if (silentRunning) return;
    bool message_changed = false;
    BinaryMessage message(messageLabel);

    uint16_t new_voltage = talonStatus->bus_voltage * 100.0;
    uint16_t new_current = talonStatus->output_current * 100.0;
    float new_sensor_pos = talonStatus->sensor_position;

    update_if_changed(message, message_changed, talon.voltage,         new_voltage,                   Field_Strings::BusVoltage);
    update_if_changed(message, message_changed, talon.current,         new_current,                   Field_Strings::OutputCurrent);
    update_if_changed(message, message_changed, talon.output_percent,  talonStatus->output_percent,   Field_Strings::OutputPercent);
    update_if_changed(message, message_changed, talon.temperature,     (uint8_t)talonStatus->temperature, Field_Strings::Temperature);
    update_if_changed(message, message_changed, talon.sensor_position, new_sensor_pos,                Field_Strings::SensorPosition);
    update_if_changed(message, message_changed, talon.sensor_velocity, talonStatus->sensor_velocity,  Field_Strings::SensorVelocity);
    update_if_changed(message, message_changed, talon.max_current,     talonStatus->max_current,      Field_Strings::MaxCurrent);
    ForceBoolFlags& tf = getForceFlags(talon);
    update_bool_with_force(message, message_changed, talon.temp_disable, talonStatus->temp_disable, tf.temp_disable, Field_Strings::TempDisable);

    if (message_changed) send(message);
}

void send(std::string messageLabel, const messages::msg::Power::SharedPtr power) {
    if (silentRunning) return;

    const std::array<float, 16> power_currents = {
        power->current0,  power->current1,  power->current2,  power->current3,
        power->current4,  power->current5,  power->current6,  power->current7,
        power->current8,  power->current9,  power->current10, power->current11,
        power->current12, power->current13, power->current14, power->current15
    };

    bool message1_changed = false;
    BinaryMessage message1(messageLabel);
    update_if_changed(message1, message1_changed, voltage, power->voltage, Field_Strings::Voltage);
    update_if_changed(message1, message1_changed, temperature_val, power->temperature, Field_Strings::Temp);
    for (int i = 0; i <= 6; ++i)
        update_if_changed(message1, message1_changed, currents[i], power_currents[i], "Current " + std::to_string(i));
    if (message1_changed) send(message1);

    bool message2_changed = false;
    BinaryMessage message2("Power2");
    for (int i = 7; i <= 15; ++i)
        update_if_changed(message2, message2_changed, currents[i], power_currents[i], "Current " + std::to_string(i));
    if (message2_changed) send(message2);
}

void send(std::string messageLabel, const messages::msg::LinearStatus::SharedPtr linearStatus, Linear& linear) {
    if (silentRunning) return;
    bool message_changed = false;
    BinaryMessage message(messageLabel);

    update_if_changed(message, message_changed, linear.motor_number,        (uint8_t)linearStatus->motor_number,      Field_Strings::MotorNumber);
    update_if_changed(message, message_changed, linear.speed,               linearStatus->speed,                      Field_Strings::Speed);
    update_if_changed(message, message_changed, linear.potentiometer,       (uint16_t)linearStatus->potentiometer,    Field_Strings::Potentiometer);
    update_if_changed(message, message_changed, linear.time_without_change, (uint8_t)linearStatus->time_without_change, Field_Strings::TimeWithoutChange);
    update_if_changed(message, message_changed, linear.max,                 (uint16_t)linearStatus->max,              Field_Strings::Max);
    update_if_changed(message, message_changed, linear.min,                 (uint16_t)linearStatus->min,              Field_Strings::Min);
    update_if_changed(message, message_changed, linear.error,               linearStatus->error,                      Field_Strings::Error);
    update_if_changed(message, message_changed, linear.at_min,              linearStatus->at_min,                     Field_Strings::AtMin);
    update_if_changed(message, message_changed, linear.at_max,              linearStatus->at_max,                     Field_Strings::AtMax);
    update_if_changed(message, message_changed, linear.distance,            linearStatus->distance,                   Field_Strings::Distance);
    update_if_changed(message, message_changed, linear.sensorless,          linearStatus->sensorless,                 Field_Strings::Sensorless);

    if (message_changed) send(message);
}

void send(std::string messageLabel, const messages::msg::AutonomyStatus::SharedPtr autonomy) {
    if (silentRunning) return;
    bool message_changed = false;
    BinaryMessage message(messageLabel);

    update_if_changed(message, message_changed, autonomyState.robot_state,       autonomy->robot_state,       Field_Strings::RobotState);
    update_if_changed(message, message_changed, autonomyState.excavation_state,  autonomy->excavation_state,  Field_Strings::ExcavationState);
    update_if_changed(message, message_changed, autonomyState.error_state,       autonomy->error_state,       Field_Strings::ErrorState);
    update_if_changed(message, message_changed, autonomyState.diagnostics_state, autonomy->diagnostics_state, Field_Strings::DiagnosticsState);
    update_if_changed(message, message_changed, autonomyState.tilt_state,        autonomy->tilt_state,        Field_Strings::TiltState);
    update_if_changed(message, message_changed, autonomyState.dump_state,        autonomy->dump_state,        Field_Strings::DumpState);
    update_if_changed(message, message_changed, autonomyState.bucket_state,      autonomy->bucket_state,      Field_Strings::LevelBucket);
    update_if_changed(message, message_changed, autonomyState.arms_state,        autonomy->arms_state,        Field_Strings::LevelArms);
    update_if_changed(message, message_changed, autonomyState.dest_x,            autonomy->dest_x,            Field_Strings::DestX);
    update_if_changed(message, message_changed, autonomyState.dest_z,            autonomy->dest_z,            Field_Strings::DestZ);

    if (message_changed) send(message);
}

int zedCounter = 0;
void zedPositionCallback(const messages::msg::ZedPosition::SharedPtr zedPosition) {
    if (silentRunning) return;
    if (rssi > UPPER_THRESH) return;
    zedCounter++;
    if (zedCounter % 15 != 0) return;

    bool message_changed = false;
    BinaryMessage message("Zed");
    update_if_changed(message, message_changed, zedState.x,     zedPosition->x,             Field_Strings::X);
    update_if_changed(message, message_changed, zedState.y,     zedPosition->y,             Field_Strings::Y);
    update_if_changed(message, message_changed, zedState.z,     zedPosition->z,             Field_Strings::Z);
    update_if_changed(message, message_changed, zedState.roll,  zedPosition->roll,          Field_Strings::Roll);
    update_if_changed(message, message_changed, zedState.pitch, zedPosition->pitch,         Field_Strings::Pitch);
    update_if_changed(message, message_changed, zedState.yaw,   zedPosition->yaw,           Field_Strings::Yaw);
    update_if_changed(message, message_changed, zedState.aruco, zedPosition->aruco_visible, Field_Strings::Aruco);
    if (message_changed) send(message);
}

int systemCounter = 0;
void systemStatusCallback(const messages::msg::SystemStatus::SharedPtr status) {
    if (silentRunning) return;
    systemCounter++;
    if (systemCounter % 5 != 0) return;

    bool message_changed = false;
    BinaryMessage message("Communication");

    std::string new_wifi_status;
    if (status->rssi < LOWER_THRESH) new_wifi_status = "NORMAL";
    else if (status->rssi < UPPER_THRESH) new_wifi_status = "DEGRADED";
    else if (status->rssi < CRIT_THRESH) new_wifi_status = "INTERFERENCE";
    else new_wifi_status = "NON-FUNCTIONAL";

    update_if_changed(message, message_changed, systemState.rssi,         status->rssi,          Field_Strings::RSSI);
    update_if_changed(message, message_changed, systemState.wifi,         new_wifi_status,       Field_Strings::WiFi);
    update_if_changed(message, message_changed, systemState.can_bus,      status->can_message,   Field_Strings::CANBus);
    update_if_changed(message, message_changed, systemState.using_can1,   status->using_can1,    Field_Strings::UsingCan1);
    update_if_changed(message, message_changed, systemState.rx_packets,   status->rx_packets,    Field_Strings::RXPackets);
    update_if_changed(message, message_changed, systemState.tx_packets,   status->tx_packets,    Field_Strings::TXPackets);
    update_if_changed(message, message_changed, systemState.can_bus2,     status->can2_message,  Field_Strings::CANBus2);
    update_if_changed(message, message_changed, systemState.rx_packets2,  status->rx2_packets,   Field_Strings::RX2Packets);
    update_if_changed(message, message_changed, systemState.tx_packets2,  status->tx2_packets,   Field_Strings::TX2Packets);
    update_if_changed(message, message_changed, systemState.first_motor,  status->first_motor,   Field_Strings::FirstMotor);
    update_if_changed(message, message_changed, systemState.second_motor, status->second_motor,  Field_Strings::SecondMotor);
    update_if_changed(message, message_changed, systemState.num_breaks,   status->num_breaks,    Field_Strings::NumBreaks);
    if (message_changed) send(message);
}

int drivetrainCounter = 0;
void drivetrainStatusCallback(const messages::msg::DrivetrainStatus::SharedPtr status) {
    if (silentRunning) return;
    drivetrainCounter++;
    if (drivetrainCounter % 10 != 0) return;
    bool message_changed = false;
    BinaryMessage message("Drivetrain");

    update_if_changed(message, message_changed, drivetrainState.f1_vel,   status->falcon1_velocity,      Field_Strings::F1Vel);
    update_if_changed(message, message_changed, drivetrainState.f1_rpm,   status->falcon1_rpm,           Field_Strings::F1RPM);
    update_if_changed(message, message_changed, drivetrainState.f1_speed, status->falcon1_ground_speed,  Field_Strings::F1Speed);
    update_if_changed(message, message_changed, drivetrainState.f2_vel,   status->falcon2_velocity,      Field_Strings::F2Vel);
    update_if_changed(message, message_changed, drivetrainState.f2_rpm,   status->falcon2_rpm,           Field_Strings::F2RPM);
    update_if_changed(message, message_changed, drivetrainState.f2_speed, status->falcon2_ground_speed,  Field_Strings::F2Speed);
    update_if_changed(message, message_changed, drivetrainState.f3_vel,   status->falcon3_velocity,      Field_Strings::F3Vel);
    update_if_changed(message, message_changed, drivetrainState.f3_rpm,   status->falcon3_rpm,           Field_Strings::F3RPM);
    update_if_changed(message, message_changed, drivetrainState.f3_speed, status->falcon3_ground_speed,  Field_Strings::F3Speed);
    update_if_changed(message, message_changed, drivetrainState.f4_vel,   status->falcon4_velocity,      Field_Strings::F4Vel);
    update_if_changed(message, message_changed, drivetrainState.f4_rpm,   status->falcon4_rpm,           Field_Strings::F4RPM);
    update_if_changed(message, message_changed, drivetrainState.f4_speed, status->falcon4_ground_speed,  Field_Strings::F4Speed);
    if (message_changed) send(message);
}

void lidarDistanceCallback(const messages::msg::LidarDistance::SharedPtr msg) {
    if (silentRunning) return;
    
    bool message_changed = false;
    BinaryMessage message("Lidar");
    
    update_if_changed(message, message_changed, current_lidar_dist, msg->distance_m, "Distance");

    if (message_changed) send(message);
}

int powerCounter = 0;
void powerCallback(const messages::msg::Power::SharedPtr power) {
    powerCounter++;
    if (powerCounter % 5 == 0)
        if (rssi < UPPER_THRESH)
            send("Power", power);
}

void talonStatusCallback(const std::string& name, const messages::msg::TalonStatus::SharedPtr talonStatus, int& counter, Talon& talon, int motorId) {
    notifyMotorReceived(motorId);
    if (rssi < CRIT_THRESH)
        send(name, talonStatus, talon);
}

void sendFalconCrit(std::string messageLabel, const messages::msg::FalconStatus::SharedPtr talonStatus, Falcon& falcon) {
    if (silentRunning) return;
    if (talonStatus->output_percent == falcon.output_percent) return;
    BinaryMessage message(messageLabel);
    message.addElementFloat32("Output Percent", talonStatus->output_percent);
    falcon.output_percent = talonStatus->output_percent;  // keep cache in sync with what we just sent
    send(message);
}

void falconStatusCallback(const std::string& name, const messages::msg::FalconStatus::SharedPtr talonStatus, int& counter, Falcon& falcon, int motorId) {
    notifyMotorReceived(motorId);
    counter++;
    if (counter % 20 == 0) {
        if (rssi < CRIT_THRESH)
            send(name, talonStatus, falcon);
    }
    else {
        if (rssi < CRIT_THRESH)
            sendFalconCrit(name, talonStatus, falcon);
    }
}

void krakenStatusCallback(const std::string& name, const messages::msg::KrakenStatus::SharedPtr krakenStatus, int& counter, Kraken& kraken, int motorId) {
    notifyMotorReceived(motorId);
    if (rssi < CRIT_THRESH)
        send(name, krakenStatus, kraken);
}

void linearStatusCallback(const std::string& name, const messages::msg::LinearStatus::SharedPtr linearStatus,
                          int& counter, Linear& linear) {
    counter++;
    if (counter % 30 == 0)
        if (rssi < UPPER_THRESH)
            send(name, linearStatus, linear);
}

int autonomyCounter = 0;
void autonomyStatusCallback(const messages::msg::AutonomyStatus::SharedPtr autonomyStatus) {
    autonomyCounter++;
    if (autonomyCounter % 15 == 0)
        if (rssi < CRIT_THRESH)
            send("Autonomy", autonomyStatus);
}

void broadcastIP() {
    while (true) {
        if (broadcast) {
            std::string addressString = getAddressString(AF_INET, interfaceName);
            std::string message(robotName + "@" + addressString);

            int socketDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
            if (socketDescriptor >= 0) {
                struct sockaddr_in socketAddress;
                socketAddress.sin_family = AF_INET;
                socketAddress.sin_addr.s_addr = inet_addr("226.1.1.1");
                socketAddress.sin_port = htons(4321);

                struct in_addr localInterface;
                localInterface.s_addr = inet_addr(addressString.c_str());
                if (setsockopt(socketDescriptor, IPPROTO_IP, IP_MULTICAST_IF,
                               (char*)&localInterface, sizeof(localInterface)) >= 0) {
                    sendto(socketDescriptor, message.c_str(), message.length(), 0,
                           (struct sockaddr*)&socketAddress, (socklen_t)sizeof(socketAddress));
                }
            }
            close(socketDescriptor);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    nodeHandle = rclcpp::Node::make_shared("communication");
    RCLCPP_INFO(nodeHandle->get_logger(), "Starting unified communication node");

    // --- Parameters ---
    robotName       = utils::getParameter<std::string>(nodeHandle, "robot_name", "shovel");
    debug           = utils::getParameter<bool>(nodeHandle, "debug", false);
    interfaceName   = utils::getParameter<std::string>(nodeHandle, "interface_name", "wlP1p1s0");

    RCLCPP_INFO(nodeHandle->get_logger(), "Comms Thread Started.");

    // --- Publishers ---
    auto joystickAxisPublisher   = nodeHandle->create_publisher<messages::msg::AxisState>("joystick_axis", 1);
    auto joystickHatPublisher    = nodeHandle->create_publisher<messages::msg::HatState>("joystick_hat", 1);
    auto joystickButtonPublisher = nodeHandle->create_publisher<messages::msg::ButtonState>("joystick_button", 1);
    auto keyPublisher            = nodeHandle->create_publisher<messages::msg::KeyState>("key", 1);
    auto stopPublisher           = nodeHandle->create_publisher<std_msgs::msg::Empty>("STOP", 1);
    auto goPublisher             = nodeHandle->create_publisher<std_msgs::msg::Empty>("GO", 1);
    auto commHeartbeatPublisher  = nodeHandle->create_publisher<std_msgs::msg::Empty>("comm_heartbeat", 1);

    auto powerSubscriber = nodeHandle->create_subscription<messages::msg::Power>("power", 1, powerCallback);

    int talon1Counter = 0, talon2Counter = 0, talon3Counter = 0, talon4Counter = 0;
    auto talon1Sub = nodeHandle->create_subscription<messages::msg::TalonStatus>("talon_14_info", 1,
        [&](const messages::msg::TalonStatus::SharedPtr msg) { talonStatusCallback("Talon 1", msg, talon1Counter, talon1, 14); });
    auto talon2Sub = nodeHandle->create_subscription<messages::msg::TalonStatus>("talon_15_info", 1,
        [&](const messages::msg::TalonStatus::SharedPtr msg) { talonStatusCallback("Talon 2", msg, talon2Counter, talon2, 15); });
    auto talon3Sub = nodeHandle->create_subscription<messages::msg::TalonStatus>("talon_16_info", 1,
        [&](const messages::msg::TalonStatus::SharedPtr msg) { talonStatusCallback("Talon 3", msg, talon3Counter, talon3, 16); });
    auto talon4Sub = nodeHandle->create_subscription<messages::msg::TalonStatus>("talon_17_info", 1,
        [&](const messages::msg::TalonStatus::SharedPtr msg) { talonStatusCallback("Talon 4", msg, talon4Counter, talon4, 17); });

    std::string motor10Type = utils::getParameter<std::string>(nodeHandle, "motor10_type", "falcon");
    std::string motor11Type = utils::getParameter<std::string>(nodeHandle, "motor11_type", "falcon");
    std::string motor12Type = utils::getParameter<std::string>(nodeHandle, "motor12_type", "falcon");
    std::string motor13Type = utils::getParameter<std::string>(nodeHandle, "motor13_type", "falcon");

    int falcon1Counter = 0, falcon2Counter = 0, falcon3Counter = 0, falcon4Counter = 0;
    int kraken1Counter = 0, kraken2Counter = 0, kraken3Counter = 0, kraken4Counter = 0;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> driveMotorSubs;

    // Motor 10 (right front)
    if (motor10Type == "kraken") {
        driveMotorSubs.push_back(nodeHandle->create_subscription<messages::msg::KrakenStatus>("talon_10_info", 1,
            [&](const messages::msg::KrakenStatus::SharedPtr msg) { krakenStatusCallback("Kraken 1", msg, kraken1Counter, kraken1, 10); }));
    } else {
        driveMotorSubs.push_back(nodeHandle->create_subscription<messages::msg::FalconStatus>("talon_10_info", 1,
            [&](const messages::msg::FalconStatus::SharedPtr msg) { falconStatusCallback("Falcon 1", msg, falcon1Counter, falcon1, 10); }));
    }

    // Motor 11 (left front)
    if (motor11Type == "kraken") {
        driveMotorSubs.push_back(nodeHandle->create_subscription<messages::msg::KrakenStatus>("talon_11_info", 1,
            [&](const messages::msg::KrakenStatus::SharedPtr msg) { krakenStatusCallback("Kraken 2", msg, kraken2Counter, kraken2, 11); }));
    } else {
        driveMotorSubs.push_back(nodeHandle->create_subscription<messages::msg::FalconStatus>("talon_11_info", 1,
            [&](const messages::msg::FalconStatus::SharedPtr msg) { falconStatusCallback("Falcon 2", msg, falcon2Counter, falcon2, 11); }));
    }

    // Motor 12 (right rear)
    if (motor12Type == "kraken") {
        driveMotorSubs.push_back(nodeHandle->create_subscription<messages::msg::KrakenStatus>("talon_12_info", 1,
            [&](const messages::msg::KrakenStatus::SharedPtr msg) { krakenStatusCallback("Kraken 3", msg, kraken3Counter, kraken3, 12); }));
    } else {
        driveMotorSubs.push_back(nodeHandle->create_subscription<messages::msg::FalconStatus>("talon_12_info", 1,
            [&](const messages::msg::FalconStatus::SharedPtr msg) { falconStatusCallback("Falcon 3", msg, falcon3Counter, falcon3, 12); }));
    }

    // Motor 13 (left rear)
    if (motor13Type == "kraken") {
        driveMotorSubs.push_back(nodeHandle->create_subscription<messages::msg::KrakenStatus>("talon_13_info", 1,
            [&](const messages::msg::KrakenStatus::SharedPtr msg) { krakenStatusCallback("Kraken 4", msg, kraken4Counter, kraken4, 13); }));
    } else {
        driveMotorSubs.push_back(nodeHandle->create_subscription<messages::msg::FalconStatus>("talon_13_info", 1,
            [&](const messages::msg::FalconStatus::SharedPtr msg) { falconStatusCallback("Falcon 4", msg, falcon4Counter, falcon4, 13); }));
    }

    RCLCPP_INFO(nodeHandle->get_logger(), "Drive motor types: [%s, %s, %s, %s]",
                motor10Type.c_str(), motor11Type.c_str(), motor12Type.c_str(), motor13Type.c_str());

    motorStopPublishers[0] = nodeHandle->create_publisher<std_msgs::msg::Bool>("falcon_10_stop", 1);
    motorStopPublishers[1] = nodeHandle->create_publisher<std_msgs::msg::Bool>("falcon_11_stop", 1);
    motorStopPublishers[2] = nodeHandle->create_publisher<std_msgs::msg::Bool>("falcon_12_stop", 1);
    motorStopPublishers[3] = nodeHandle->create_publisher<std_msgs::msg::Bool>("falcon_13_stop", 1);
    motorStopPublishers[4] = nodeHandle->create_publisher<std_msgs::msg::Bool>("talon_14_stop", 1);
    motorStopPublishers[5] = nodeHandle->create_publisher<std_msgs::msg::Bool>("talon_15_stop", 1);
    motorStopPublishers[6] = nodeHandle->create_publisher<std_msgs::msg::Bool>("talon_16_stop", 1);
    motorStopPublishers[7] = nodeHandle->create_publisher<std_msgs::msg::Bool>("talon_17_stop", 1);

    int linear1Counter = 0, linear2Counter = 0, linear3Counter = 0, linear4Counter = 0;
    auto linear1Sub = nodeHandle->create_subscription<messages::msg::LinearStatus>("linearStatus1", 1,
        [&](const messages::msg::LinearStatus::SharedPtr msg) { linearStatusCallback("Linear 1", msg, linear1Counter, linear1); });
    auto linear2Sub = nodeHandle->create_subscription<messages::msg::LinearStatus>("linearStatus2", 1,
        [&](const messages::msg::LinearStatus::SharedPtr msg) { linearStatusCallback("Linear 2", msg, linear2Counter, linear2); });
    auto linear3Sub = nodeHandle->create_subscription<messages::msg::LinearStatus>("linearStatus3", 1,
        [&](const messages::msg::LinearStatus::SharedPtr msg) { linearStatusCallback("Linear 3", msg, linear3Counter, linear3); });
    auto linear4Sub = nodeHandle->create_subscription<messages::msg::LinearStatus>("linearStatus4", 1,
        [&](const messages::msg::LinearStatus::SharedPtr msg) { linearStatusCallback("Linear 4", msg, linear4Counter, linear4); });

    auto zedSub       = nodeHandle->create_subscription<messages::msg::ZedPosition>("zed_position", 1, zedPositionCallback);
    auto autonomySub  = nodeHandle->create_subscription<messages::msg::AutonomyStatus>("autonomy_status", 10, autonomyStatusCallback);
    auto systemSub    = nodeHandle->create_subscription<messages::msg::SystemStatus>("system_status", 10, systemStatusCallback);
    auto drivetrainSub = nodeHandle->create_subscription<messages::msg::DrivetrainStatus>("drivetrain_status", 10, drivetrainStatusCallback);
    auto lidarSub = nodeHandle->create_subscription<messages::msg::LidarDistance>("lidar_distance", 1, lidarDistanceCallback);

    // --- Socket setup ---
    int server_fd, bytesRead;
    int opt = 1;
    uint8_t buffer[1024] = {0};
    std::string hello("Hello from server");
    std::thread broadcastThread(broadcastIP);
    broadcastThread.detach();

    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    new_socket = server_fd;

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    if (bindAddr == "0.0.0.0") {
        address.sin_addr.s_addr = INADDR_ANY;
    }
    else {
        inet_pton(AF_INET, bindAddr.c_str(), &address.sin_addr);
    }
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    fcntl(server_fd, F_SETFL, O_NONBLOCK);
    bool connected = false;

    while (!connected) {
        bytesRead = recvfrom(server_fd, buffer, 1024, 0, (struct sockaddr*)&address, &addrlen);
        if (bytesRead > 0) {
            sendto(server_fd, hello.c_str(), strlen(hello.c_str()), 0,
                   (struct sockaddr*)&address, addrlen);
            silentRunning = true;
            broadcast = false;
            connected = true;
        }
        rclcpp::spin_some(nodeHandle);
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
    }
    last_client_tx_time_ms.store(get_time_ms(), std::memory_order_relaxed);

    std::list<uint8_t> messageBytesList;
    uint8_t message[256];
    rclcpp::Rate rate(120);
    bool isClientConnected = false; // Orin starts connected, Nano/Sim starts disconnected
    auto previousHeartbeat = std::chrono::high_resolution_clock::now();
    auto previousReset = std::chrono::high_resolution_clock::now();

    while (rclcpp::ok()) {
        last_ros_update_time = get_time_ms();

        try {
            bytesRead = recvfrom(server_fd, buffer, 1024, 0, (struct sockaddr*)&address, &addrlen);
            for (int index = 0; index < bytesRead; index++) {
                messageBytesList.push_back(buffer[index]);
            }
            if (debug && bytesRead != -1) {
                RCLCPP_INFO(nodeHandle->get_logger(), "Bytes Read: %d", bytesRead);
            }
        }
        catch (int x) {
            RCLCPP_INFO(nodeHandle->get_logger(), "ERROR: Exception reading data from client");
        }

        if (bytesRead > 0) {
            if (!isClientConnected) {
                RCLCPP_INFO(nodeHandle->get_logger(), "New client connected. Sending greeting.");
                isClientConnected = true;
                previousHeartbeat = std::chrono::high_resolution_clock::now();
                std::string hello("Hello from server");
                sendto(server_fd, hello.c_str(), hello.length(), 0,
                       (struct sockaddr*)&address, addrlen);
                broadcast = false;
                last_client_tx_time_ms.store(get_time_ms(), std::memory_order_relaxed);
                messageBytesList.clear();
                forceDataResync();
            }
        }

        if (isClientConnected) {
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = now - previousHeartbeat;

            if (elapsed.count() > 5.0) {
                isClientConnected = false;
                RCLCPP_INFO(nodeHandle->get_logger(), "Client disconnected");
                silentRunning = true;
                broadcast = true;
            }

            elapsed = now - previousReset;
            if (elapsed.count() > 5) {
                forceDataResync();
                previousReset = now;
            }

            uint64_t now_ms = get_time_ms();
            uint64_t last_tx = last_client_tx_time_ms.load(std::memory_order_relaxed);
            if (last_tx == 0) last_client_tx_time_ms.store(now_ms, std::memory_order_relaxed);
            if (now_ms - last_tx > 500) {
                sendClientHeartbeat();
            }
        }

        // --- Message parsing ---
        while (messageBytesList.size() > 0 && messageBytesList.front() <= messageBytesList.size()) {
            int messageSize = messageBytesList.front();
            messageBytesList.pop_front();
            messageSize--;
            for (int index = 0; index < messageSize; index++) {
                message[index] = messageBytesList.front();
                messageBytesList.pop_front();
            }

            uint8_t command = message[0];
            if (debug) {
                RCLCPP_INFO(nodeHandle->get_logger(), "Message size: %d, Command: %d", messageSize, command);
            }

            // Heartbeat
            if (command == 0) {
                previousHeartbeat = std::chrono::high_resolution_clock::now();
                if (debug) RCLCPP_INFO(nodeHandle->get_logger(), "Received heartbeat");
            }

            // Joystick axis
            if (command == 1) {
                messages::msg::AxisState axisState;
                axisState.joystick = message[1];
                axisState.axis = message[2];
                axisState.state = -parseFloat(&message[3]);
                joystickAxisPublisher->publish(axisState);
            }

            // Key state
            if (command == 2) {
                messages::msg::KeyState keyState;
                keyState.key = ((uint16_t)message[1]) << 8 | ((uint16_t)message[2]);
                keyState.state = message[3];
                keyPublisher->publish(keyState);
            }

            // Joystick button
            if (command == 5) {
                messages::msg::ButtonState buttonState;
                buttonState.joystick = message[1];
                buttonState.button = message[2];
                buttonState.state = message[3];
                if (buttonState.button == 0 && buttonState.state == 0) {
                    std::cout << "publish stop" << std::endl;
                    stopPublisher->publish(empty);
                }
                if (buttonState.button == 0 && buttonState.state == 1) {
                    std::cout << "publish go" << std::endl;
                    goPublisher->publish(empty);
                }
                joystickButtonPublisher->publish(buttonState);
            }

            // Joystick hat
            if (command == 6) {
                messages::msg::HatState hatState;
                hatState.joystick = message[1];
                hatState.hat = message[2];
                hatState.state = message[3];
                joystickHatPublisher->publish(hatState);
            }

            // Silent running toggle
            if (command == 7) {
                silentRunning = message[1];
                std::cout << "silentRunning " << silentRunning << std::endl;
            }

            // Reboot
            if (command == 8) {
                reboot();
                std::cout << "reboot" << std::endl;
            }
        }

        rclcpp::spin_some(nodeHandle);
        commHeartbeatPublisher->publish(heartbeat);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}