#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <typeinfo>
#include <cmath>

#include <unistd.h>
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
#include <linux/reboot.h>
#include <sys/reboot.h>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <messages/msg/key_state.hpp>
#include <std_msgs/msg/string.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include <ctre/Phoenix.h>
#include <ctre/phoenix/platform/Platform.h>
#include <ctre/phoenix/unmanaged/Unmanaged.h>
#include <ctre/phoenix/cci/Unmanaged_CCI.h>
#include <ctre/phoenix/cci/Diagnostics_CCI.h>

#include "messages/msg/falcon_status.hpp"
#include "utils/utils.hpp"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

rclcpp::Node::SharedPtr nodeHandle;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String_<std::allocator<void> >, std::allocator<void> > > resetPublisher;
bool GO=false;
std::chrono::time_point<std::chrono::high_resolution_clock> commPrevious;
std::chrono::time_point<std::chrono::high_resolution_clock> logicPrevious;
TalonFX* talonFX;
bool TEMP_DISABLE = false;
float Speed = 0.0;
bool error = false;
bool restarted = false;

// Operating modes:
// 0 - Normal
// 1 - Critical
// 2 - Emergency 
int op_mode = 0;
int killKey = 0;
bool printData = false;
int errorCounter = 0;
std::string resetString = "";

// --- Simulation Variables ---
double simPosition = 0.0;
double simVelocity = 0.0;
double simCurrent = 0.0;
// Falcon 500 approx: 2048 units/rev, ~6380 RPM free speed
const double MAX_RPM = 6380.0;
const double UNITS_PER_REV = 2048.0;
const double MAX_VELOCITY_UNITS = (MAX_RPM / 60.0) * UNITS_PER_REV * 0.1; // Units per 100ms
const double SIM_LOOP_PERIOD = 0.02; // 20ms loop

void stopCallback(std_msgs::msg::Empty::SharedPtr empty){
	if(printData) RCLCPP_INFO(nodeHandle->get_logger(),"STOP");
	GO=false;
	talonFX->Set(ControlMode::PercentOutput, 0.0);
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

void speedCallback(const std_msgs::msg::Float32::SharedPtr speed){
	if(printData) RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
	if(speed->data != Speed){
		talonFX->Set(ControlMode::PercentOutput, speed->data);
		Speed = speed->data;
	}
}

void userSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
	if(printData) RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
	if(speed->data != Speed){
		double targetVelocity_RPM = 6000 * speed->data; 
		talonFX->Set(ControlMode::Velocity, targetVelocity_RPM * 2048 / 600.0);
		Speed = speed->data;
	}
}

void checkTemperature(double temperature){
	double limit = 70.0;
    if (op_mode == 1) limit = 80.0;
    if (op_mode == 2) limit = 90.0;
    TEMP_DISABLE = (temperature > limit);
}

void keyCallback(const messages::msg::KeyState::SharedPtr keyState){
    if(printData) std::cout << "Key " << keyState->key << " " << keyState->state << std::endl;
	if(keyState->key == 98 && keyState->state==1){
		std_msgs::msg::String reset;
		reset.data = resetString;
		resetPublisher->publish(reset);
	}
}

// --- Physics Simulation Function ---
void updateSimPhysics() {
    TalonFXSimCollection& sim = talonFX->GetSimCollection();

    // 1. Set Input Voltage (Battery)
    sim.SetBusVoltage(12.0);

    // 2. Read Output Voltage
    double motorVoltage = sim.GetMotorOutputLeadVoltage();
    double motorPercent = motorVoltage / 12.0;

    // 3. Calculate Velocity (Simple DC Motor Model: Velocity proportional to Voltage)
    // In reality, this should account for inertia, back-EMF, etc.
    // Ideally: V_target = motorPercent * MAX_VELOCITY_UNITS
    double targetVelocity = motorPercent * MAX_VELOCITY_UNITS;

    // Simple inertia: ramp simulation velocity towards target
    double ramp = MAX_VELOCITY_UNITS * 0.05; // 5% speed change per loop
    if (simVelocity < targetVelocity) {
        simVelocity += ramp;
        if (simVelocity > targetVelocity) simVelocity = targetVelocity;
    } else {
        simVelocity -= ramp;
        if (simVelocity < targetVelocity) simVelocity = targetVelocity;
    }

    // 4. Calculate Position
    // Velocity is in units/100ms. Loop is ~20ms.
    // ticks = velocity * (loop_time / 0.1s)
    double positionChange = simVelocity * (SIM_LOOP_PERIOD / 0.1);
    simPosition += positionChange;

    // 5. Calculate Current (Simulated Load)
    simCurrent = std::abs(motorPercent) * 5.0; // Arbitrary 5A at full speed

    // 6. Push values back to SimCollection
    sim.SetIntegratedSensorRawPosition((int)simPosition);
    sim.SetIntegratedSensorVelocity((int)simVelocity);
    sim.SetSupplyCurrent(simCurrent);
}

int main(int argc,char** argv){
	rclcpp::init(argc,argv);
	nodeHandle = rclcpp::Node::make_shared("talon");

	RCLCPP_INFO(nodeHandle->get_logger(),"Starting SIMULATED Falcon");

	int motorNumber = utils::getParameter<int>(nodeHandle, "motor_number", 1);
	int portNumber = utils::getParameter<int>(nodeHandle, "diagnostics_port", 1);
    
    // In Simulation, we don't strictly need the Diagnostics server, but we keep the structure
	// c_Phoenix_Diagnostics_Create1(portNumber); 

	std::string infoTopic = utils::getParameter<std::string>(nodeHandle, "info_topic", "unset");
	std::string speedTopic = utils::getParameter<std::string>(nodeHandle, "speed_topic", "unset");
	std::string userTopic = utils::getParameter<std::string>(nodeHandle, "user_topic", "unset");
	resetString = utils::getParameter<std::string>(nodeHandle, "reset_topic", "1");
	bool invertMotor = utils::getParameter<bool>(nodeHandle, "invert_motor", false);
    // PID constants
	double kP = utils::getParameter<double>(nodeHandle, "kP", 1.0);
	double kI = utils::getParameter<double>(nodeHandle, "kI", 0.0);
	double kD = utils::getParameter<double>(nodeHandle, "kD", 0.0);
	double kF = utils::getParameter<double>(nodeHandle, "kF", 0.0);
	int publishingDelay = utils::getParameter<int>(nodeHandle, "publishing_delay", 0);
	op_mode = utils::getParameter<int>(nodeHandle, "op_mode", 0);
	printData = utils::getParameter<bool>(nodeHandle, "print_data", false);

    // Note: We skip SetCANInterface for local simulation if not using a vcan
	// ctre::phoenix::platform::can::SetCANInterface(can_interface.c_str());

	int kTimeoutMs=30;
	int kPIDLoopIdx=0;
	talonFX=new TalonFX(motorNumber);
	RCLCPP_INFO(nodeHandle->get_logger(),"created talon instance");

	if(invertMotor){
		talonFX->SetInverted(TalonFXInvertType::CounterClockwise);
	}
	else{
		talonFX->SetInverted(TalonFXInvertType::Clockwise);
	}
	talonFX->SelectProfileSlot(0,0);
	talonFX->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
	talonFX->ConfigClosedloopRamp(2);
    
    // PID Config
	talonFX->Config_kF(kPIDLoopIdx, kF, kTimeoutMs);
	talonFX->Config_kP(kPIDLoopIdx, kP, kTimeoutMs);
	talonFX->Config_kI(kPIDLoopIdx, kI, kTimeoutMs);
	talonFX->Config_kD(kPIDLoopIdx, kD, kTimeoutMs);
	talonFX->ConfigAllowableClosedloopError(kPIDLoopIdx,0,kTimeoutMs);

	talonFX->Set(ControlMode::PercentOutput, 0);

	TalonFXConfiguration allConfigs;
	ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration supplyLimitConfig;
    supplyLimitConfig.enable = true;
    supplyLimitConfig.currentLimit = 70.0;
    talonFX->ConfigSupplyCurrentLimit(supplyLimitConfig, kTimeoutMs);

	messages::msg::FalconStatus falconStatus;
	auto falconStatusPublisher=nodeHandle->create_publisher<messages::msg::FalconStatus>(infoTopic.c_str(),1);
	auto speedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32>(speedTopic.c_str(),1,speedCallback);
	auto userSpeedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32>(userTopic.c_str(),1,userSpeedCallback);
	resetPublisher=nodeHandle->create_publisher<std_msgs::msg::String>("reset_topic",1);

	auto stopSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP",1,stopCallback);
	auto goSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("GO",1,goCallback);
	auto commHeartbeatSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("comm_heartbeat",1,commHeartbeatCallback);
	auto logicHeartbeatSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("logic_heartbeat",1,logicHeartbeatCallback);
	auto keySubscriber= nodeHandle->create_subscription<messages::msg::KeyState>("key",1,keyCallback);

	RCLCPP_INFO(nodeHandle->get_logger(),"set subscribers");

	rclcpp::Rate rate(50);
	auto start = std::chrono::high_resolution_clock::now();
	auto errorTimer = std::chrono::high_resolution_clock::now();
    commPrevious = start;
    logicPrevious = start;

	float maxCurrent = 0.0;
	double busVoltage = 0.0;

	while(rclcpp::ok()){
        // IMPORTANT: In simulation, we must enable the "safety" check locally
		ctre::phoenix::unmanaged::FeedEnable(100);
        
        // --- UPDATE SIMULATION PHYSICS ---
        if(GO) updateSimPhysics();

		auto finish = std::chrono::high_resolution_clock::now();

		if(error){
			if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-errorTimer).count() > 1500){
				restarted = true;
			}
		}

		if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > publishingDelay){
			int deviceID=talonFX->GetDeviceID();
			busVoltage=talonFX->GetBusVoltage();
			double outputCurrent=talonFX->GetOutputCurrent();
			double motorOutputPercent=talonFX->GetMotorOutputPercent();
            
            // Standard Error Checking
			if(Speed > 0.1 && motorOutputPercent == 0.0){
				errorCounter++;
				if(errorCounter > 5 && !error){
					RCLCPP_INFO(nodeHandle->get_logger(), "Falcon %d ERROR", deviceID);
					error = true;
					errorTimer = std::chrono::high_resolution_clock::now();
					std_msgs::msg::String reset;
					reset.data = resetString;
					resetPublisher->publish(reset);
				}
			}
			else{
				if(motorOutputPercent != 0.0){
					error = false;
					restarted = false;
					errorCounter = 0;
				}
			}
			double temperature=talonFX->GetTemperature();
			double sensorPosition0=talonFX->GetSelectedSensorPosition(0);
			double sensorVelocity0=talonFX->GetSelectedSensorVelocity(0);
		
			falconStatus.device_id=deviceID;	
			falconStatus.bus_voltage=busVoltage;
			falconStatus.output_current=outputCurrent;
			falconStatus.output_percent=motorOutputPercent;
			falconStatus.temperature=temperature;
			falconStatus.sensor_position=sensorPosition0;
			falconStatus.sensor_velocity=sensorVelocity0;
			falconStatus.temp_disable = TEMP_DISABLE;
			falconStatus.error = error;
			falconStatus.restarted = restarted;
			if(outputCurrent > maxCurrent){
				maxCurrent = outputCurrent;
			}
			falconStatus.max_current = maxCurrent;
			falconStatusPublisher->publish(falconStatus);
			start = std::chrono::high_resolution_clock::now();
			checkTemperature(temperature);
		}

		if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-commPrevious).count() > 100 ||  TEMP_DISABLE
		||	std::chrono::duration_cast<std::chrono::milliseconds>(finish-logicPrevious).count() > 100 ){
            // Watchdog Logic
			if(TEMP_DISABLE && printData) RCLCPP_INFO(nodeHandle->get_logger(),"Temp Disable");
			if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-commPrevious).count() > 100 && printData) RCLCPP_INFO(nodeHandle->get_logger(),"comm disable");
			
			talonFX->Set(ControlMode::PercentOutput, 0.0);
			GO = false;
		}
		rate.sleep();
		rclcpp::spin_some(nodeHandle);
	}
}