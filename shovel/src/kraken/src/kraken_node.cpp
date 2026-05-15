#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <typeinfo>

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
#include <linux/reboot.h>
#include <sys/reboot.h>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <messages/msg/key_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <sys/ioctl.h>
#include <net/if.h>
#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "messages/msg/kraken_status.hpp"
#include "utils/utils.hpp"

// Phoenix 6 Includes
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/VelocityDutyCycle.hpp>
#include <ctre/phoenix/unmanaged/Unmanaged.h> 
#include <ctre/phoenix6/CANBus.hpp>
#include <units/current.h>
#include <units/velocity.h>

using namespace ctre::phoenix6;

rclcpp::Node::SharedPtr nodeHandle;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String_<std::allocator<void> >, std::allocator<void> > > resetPublisher;
bool GO=false;
std::chrono::time_point<std::chrono::high_resolution_clock> commPrevious = std::chrono::high_resolution_clock::now();
std::chrono::time_point<std::chrono::high_resolution_clock> logicPrevious = std::chrono::high_resolution_clock::now();

hardware::TalonFX* talonFX; 

bool TEMP_DISABLE = false;
float Speed = 0.0;
bool error = false;
bool restarted = false;
bool publish = true;

int op_mode = 0;
int killKey = 0;
bool printData = false;
int errorCounter = 0;
std::string resetString = "";

int reset_cooldown_ms = 2000;
std::chrono::time_point<std::chrono::high_resolution_clock> lastResetTime;
bool reset_sent = false;

controls::DutyCycleOut percentOut{0.0};
controls::VelocityDutyCycle velOut{0_tps}; 

bool can_socket_bind_ok(const std::string& ifname) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return false;

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        close(s);
        return false; 
    }

    sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bool ok = (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    close(s);
    return ok;
}

void stopCallback(std_msgs::msg::Empty::SharedPtr empty){
    if(printData) RCLCPP_INFO(nodeHandle->get_logger(),"STOP");
    GO=false;
    if(publish) talonFX->SetControl(percentOut.WithOutput(0.0));
    Speed = 0.0;
} 

void goCallback(std_msgs::msg::Empty::SharedPtr empty){
    if(printData) RCLCPP_INFO(nodeHandle->get_logger(),"GO");
    GO=true;
}

void commHeartbeatCallback(std_msgs::msg::Empty::SharedPtr empty){
    commPrevious = std::chrono::high_resolution_clock::now();
}

void logicHeartbeatCallback(std_msgs::msg::Empty::SharedPtr empty){
    logicPrevious = std::chrono::high_resolution_clock::now();
}

void publishCallback(std_msgs::msg::Bool::SharedPtr pub){
    publish = pub->data;
    if(publish){
        if(printData) RCLCPP_INFO(nodeHandle->get_logger(), )
        talonFX->SetControl(percentOut.WithOutput(Speed));
    }
}

void speedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    if(printData) RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
    if(speed->data != Speed){
        if(publish) talonFX->SetControl(percentOut.WithOutput(speed->data));
        Speed = speed->data;
    }
}

void userSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    if(printData) RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
    if(speed->data != Speed){
        if(publish) talonFX->SetControl(percentOut.WithOutput(speed->data));
        Speed = speed->data;
    }
}

void checkTemperature(double temperature){
    switch(op_mode){
        case 0: temperature > 70 ? TEMP_DISABLE = true : TEMP_DISABLE = false; break;
        case 1: temperature > 80 ? TEMP_DISABLE = true : TEMP_DISABLE = false; break;
        case 2: temperature > 90 ? TEMP_DISABLE = true : TEMP_DISABLE = false; break;
    }
}

/** @brief Check all relevant Phoenix 6 fault flags.
 *
 * Returns true if any critical hardware, current, or voltage fault is active.
 */
bool isAnyFaultTripped(){
    return talonFX->GetStickyFault_SupplyCurrLimit().GetValue() ||
           talonFX->GetStickyFault_StatorCurrLimit().GetValue() ||
           talonFX->GetStickyFault_Undervoltage().GetValue() ||
           talonFX->GetStickyFault_Hardware().GetValue() ||
           talonFX->GetStickyFault_DeviceTemp().GetValue() ||
           talonFX->GetStickyFault_ProcTemp().GetValue() ||
           talonFX->GetStickyFault_BootDuringEnable().GetValue();
}

/** @brief Log the specific active faults to the ROS2 console. */
void printActiveFaults(){
    int id = talonFX->GetDeviceID();
    auto logger = nodeHandle->get_logger();

    if (talonFX->GetStickyFault_SupplyCurrLimit().GetValue()) RCLCPP_WARN(logger, "Kraken %d FAULT: Supply Current Limit Tripped", id);
    if (talonFX->GetStickyFault_StatorCurrLimit().GetValue()) RCLCPP_WARN(logger, "Kraken %d FAULT: Stator Current Limit Tripped", id);
    if (talonFX->GetStickyFault_Undervoltage().GetValue()) RCLCPP_WARN(logger, "Kraken %d FAULT: Supply Undervoltage (Battery Sag)", id);
    if (talonFX->GetStickyFault_Hardware().GetValue()) RCLCPP_WARN(logger, "Kraken %d FAULT: Hardware Failure", id);
    if (talonFX->GetStickyFault_DeviceTemp().GetValue()) RCLCPP_WARN(logger, "Kraken %d FAULT: Device Temperature (Thermal Cutoff)", id);
    if (talonFX->GetStickyFault_ProcTemp().GetValue()) RCLCPP_WARN(logger, "Kraken %d FAULT: Processor Temperature", id);
    if (talonFX->GetStickyFault_BootDuringEnable().GetValue()) RCLCPP_WARN(logger, "Kraken %d FAULT: Boot During Enable (Power Loss/Brownout)", id);
}

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("talon");

    RCLCPP_INFO(nodeHandle->get_logger(),"Starting talon");

    int motorNumber = ::utils::getParameter<int>(nodeHandle, "motor_number", 1);
    int portNumber = ::utils::getParameter<int>(nodeHandle, "diagnostics_port", 1);
    setenv("PHOENIX_DIAGNOSTICS_PORT", std::to_string(portNumber).c_str(), 1);
    std::string infoTopic = ::utils::getParameter<std::string>(nodeHandle, "info_topic", "unset");
    std::string speedTopic = ::utils::getParameter<std::string>(nodeHandle, "speed_topic", "unset");
    std::string userTopic = ::utils::getParameter<std::string>(nodeHandle, "user_topic", "unset");
    resetString = ::utils::getParameter<std::string>(nodeHandle, "reset_topic", "1");
    std::string stopTopic = ::utils::getParameter<std::string>(nodeHandle, "stop_topic", "unset");
    bool invertMotor = ::utils::getParameter<bool>(nodeHandle, "invert_motor", false);
    double kP = ::utils::getParameter<double>(nodeHandle, "kP", 1.0);
    double kI = ::utils::getParameter<double>(nodeHandle, "kI", 0.0);
    double kD = ::utils::getParameter<double>(nodeHandle, "kD", 0.0);
    double kF = ::utils::getParameter<double>(nodeHandle, "kF", 0.0);
    int publishingDelay = ::utils::getParameter<int>(nodeHandle, "publishing_delay", 0);
    killKey = ::utils::getParameter<int>(nodeHandle, "kill_key", 0);
    op_mode = ::utils::getParameter<int>(nodeHandle, "op_mode", 0);
    printData = ::utils::getParameter<bool>(nodeHandle, "print_data", false);
    std::string can_interface = ::utils::getParameter<std::string>(nodeHandle, "can_interface", "can0");

    while (rclcpp::ok() && !can_socket_bind_ok(can_interface)) {
        RCLCPP_WARN_THROTTLE(
            nodeHandle->get_logger(), *nodeHandle->get_clock(), 1000,
            "CAN interface '%s' is DOWN / not bindable. Waiting...", can_interface.c_str()
        );
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    RCLCPP_INFO(nodeHandle->get_logger(),"Opened CAN interface");

    talonFX = new hardware::TalonFX(motorNumber, ctre::phoenix6::CANBus(can_interface));
    
    RCLCPP_INFO(nodeHandle->get_logger(),"created talon instance");
    configs::TalonFXConfiguration allConfigs;

    allConfigs.Slot0.kP = kP;
    allConfigs.Slot0.kI = kI;
    allConfigs.Slot0.kD = kD;
    allConfigs.Slot0.kS = kF; 

    allConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    allConfigs.CurrentLimits.SupplyCurrentLimit = units::current::ampere_t{70.0};

    allConfigs.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    allConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5_s;
    allConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5_s;
    allConfigs.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5_s;
    allConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5_s;
    
    if(invertMotor){
        allConfigs.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
    }
    else {
        allConfigs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    }

    talonFX->GetConfigurator().Apply(allConfigs);
    talonFX->SetControl(percentOut.WithOutput(0.0));
    
    messages::msg::KrakenStatus krakenStatus;
    auto krakenStatusPublisher=nodeHandle->create_publisher<messages::msg::KrakenStatus>(infoTopic.c_str(),1);
    auto speedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32>(speedTopic.c_str(),1,speedCallback);
    auto userSpeedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32>(userTopic.c_str(),1,userSpeedCallback);
    resetPublisher=nodeHandle->create_publisher<std_msgs::msg::String>("reset_topic",1);

    auto stopSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP",1,stopCallback);
    auto goSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("GO",1,goCallback);
    auto commHeartbeatSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("comm_heartbeat",1,commHeartbeatCallback);
    auto logicHeartbeatSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("logic_heartbeat",1,logicHeartbeatCallback);
    //auto publishSubscriber = nodeHandle->create_subscription<std_msgs::msg::Bool>(stopTopic.c_str(),1,publishCallback);

    RCLCPP_INFO(nodeHandle->get_logger(),"set subscribers");

    rclcpp::Rate rate(50);
    auto start = std::chrono::high_resolution_clock::now();
    lastResetTime = std::chrono::high_resolution_clock::now();
    float maxCurrent = 0.0;
    double busVoltage = 0.0;
    
    while(rclcpp::ok()){
        if(GO) ctre::phoenix::unmanaged::FeedEnable(100);
        auto finish = std::chrono::high_resolution_clock::now();

        if(isAnyFaultTripped()){
            auto msSinceReset = std::chrono::duration_cast<std::chrono::milliseconds>(finish - lastResetTime).count();

            if(!error){
                printActiveFaults();
                error = true;
            }

            if(!reset_sent || msSinceReset > reset_cooldown_ms){
                // Clear the sticky faults so isAnyFaultTripped() can
                // return false on the next cycle if the fault is gone
                talonFX->ClearStickyFaults();

                lastResetTime = std::chrono::high_resolution_clock::now();
                reset_sent = true;
            }
        }

        if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > publishingDelay){
            int deviceID = talonFX->GetDeviceID();
            busVoltage = talonFX->GetSupplyVoltage().GetValueAsDouble();
            double outputCurrent = talonFX->GetStatorCurrent().GetValueAsDouble(); 
            double motorOutputVoltage = talonFX->GetMotorVoltage().GetValueAsDouble();
            double motorOutputPercent = talonFX->GetDutyCycle().GetValueAsDouble();
            double temperature=talonFX->GetDeviceTemp().GetValueAsDouble();
            double sensorPosition0 = talonFX->GetPosition().GetValueAsDouble(); 
            double sensorVelocity0 = talonFX->GetVelocity().GetValueAsDouble();
            int closedLoopError0 = talonFX->GetClosedLoopError().GetValueAsDouble();
            
            krakenStatus.device_id=deviceID;    
            krakenStatus.bus_voltage=busVoltage;
            krakenStatus.output_current=outputCurrent;
            krakenStatus.output_voltage=motorOutputVoltage;
            krakenStatus.output_percent=motorOutputPercent;
            krakenStatus.temperature=temperature;
            krakenStatus.sensor_position=sensorPosition0;
            krakenStatus.sensor_velocity=sensorVelocity0;
            krakenStatus.closed_loop_error=closedLoopError0;
            
            krakenStatus.temp_disable = TEMP_DISABLE;
            krakenStatus.error = error;
            krakenStatus.restarted = restarted;
            if(outputCurrent > maxCurrent){
                maxCurrent = outputCurrent;
            }
            krakenStatus.max_current = maxCurrent;
            krakenStatusPublisher->publish(krakenStatus);
            start = std::chrono::high_resolution_clock::now();
            checkTemperature(temperature);
        }

        if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-commPrevious).count() > 100 ||  TEMP_DISABLE
        ||  std::chrono::duration_cast<std::chrono::milliseconds>(finish-logicPrevious).count() > 100 ){
            if(TEMP_DISABLE){
                if(printData) RCLCPP_INFO(nodeHandle->get_logger(),"Temp Disable");
            }
            if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-commPrevious).count() > 100){
            }
            if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-logicPrevious).count() > 100){
                if(printData) RCLCPP_INFO(nodeHandle->get_logger(),"logic disable");
            }
            talonFX->SetControl(percentOut.WithOutput(0.0));
            GO = false;
        }
        rate.sleep();
        rclcpp::spin_some(nodeHandle);
    }
}