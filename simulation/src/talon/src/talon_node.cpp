#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <typeinfo>
#include <cmath>

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
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <messages/msg/key_state.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include <ctre/Phoenix.h>
#include <ctre/phoenix/platform/Platform.h>
#include <ctre/phoenix/unmanaged/Unmanaged.h>
#include <ctre/phoenix/cci/Unmanaged_CCI.h>
#include <ctre/phoenix/cci/Diagnostics_CCI.h>

#include "messages/msg/talon_status.hpp"
#include "utils/utils.hpp"
#include <cstring>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

rclcpp::Node::SharedPtr nodeHandle;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String_<std::allocator<void> >, std::allocator<void> > > resetPublisher;
bool GO=false;
std::chrono::time_point<std::chrono::high_resolution_clock> commPrevious;
std::chrono::time_point<std::chrono::high_resolution_clock> logicPrevious;
bool printData = false;
std::string resetString = "";
int motorNumber = 0;
float curr_speed = 0.0;
bool usePosition = false;

// --- Simulation Variables ---
TalonSRX* talonSRX;
double simPosition = 500.0;
double simVelocity = 0.0;
double simCurrent = 0.0;
const double MAX_ANALOG_RANGE = 1024.0; 
const double SIM_LOOP_PERIOD = 0.01; // 10ms loop

void stopCallback(std_msgs::msg::Empty::SharedPtr empty){
	if(printData) RCLCPP_INFO(nodeHandle->get_logger(),"STOP");
	GO=false;
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

bool TEMP_DISABLE = false;
int op_mode = 0;
int killKey = 0;

void speedCallback(const std_msgs::msg::Float32::SharedPtr speed){
	if(printData) RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
	talonSRX->Set(ControlMode::PercentOutput, speed->data);
	usePosition = false;
}

void positionCallback(const std_msgs::msg::Int32::SharedPtr position){
	if(printData) RCLCPP_INFO(nodeHandle->get_logger(),"Position---------->>> %d ", position->data);
	talonSRX->Set(ControlMode::Position, position->data);
	usePosition = true;
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
    TalonSRXSimCollection& sim = talonSRX->GetSimCollection();

    // 1. Set Input Voltage
    sim.SetBusVoltage(12.0);

    // 2. Read Motor Output
    double motorVoltage = sim.GetMotorOutputLeadVoltage();
    double motorPercent = motorVoltage / 12.0;

    // 3. Simple Physics
    // Velocity is proportional to voltage
    double targetVelocity = motorPercent * 50.0; // Units per tick

    // 4. Update Position
    // If Using Position Control, we effectively calculate how the motor moves towards the target
    simPosition += targetVelocity;
    
    // Bounds checking for analog pot
    if (simPosition > MAX_ANALOG_RANGE) simPosition = MAX_ANALOG_RANGE;
    if (simPosition < 0) simPosition = 0;

    simCurrent = std::abs(motorPercent) * 10.0;

    // 5. Push values back to SimCollection (Analog)
    sim.SetAnalogPosition((int)simPosition);
    sim.SetAnalogVelocity((int)targetVelocity);
    sim.SetSupplyCurrent(simCurrent);
}

int main(int argc,char** argv){
	rclcpp::init(argc,argv);
	nodeHandle = rclcpp::Node::make_shared("talon");

	RCLCPP_INFO(nodeHandle->get_logger(),"Starting SIMULATED Talon SRX");

	motorNumber = utils::getParameter<int>(nodeHandle, "motor_number", 1);
	int portNumber = utils::getParameter<int>(nodeHandle, "diagnostics_port", 1);
    
    // Disable diagnostics for simulation
	// c_Phoenix_Diagnostics_Create1(portNumber); 

	std::string infoTopic = utils::getParameter<std::string>(nodeHandle, "info_topic", "unset");
	std::string potentiometerTopic = utils::getParameter<std::string>(nodeHandle, "potentiometer_topic", "unset");
	std::string speedTopic = utils::getParameter<std::string>(nodeHandle, "speed_topic", "unset");
	std::string positionTopic = utils::getParameter<std::string>(nodeHandle, "position_topic", "unset");

	bool invertMotor = utils::getParameter<bool>(nodeHandle, "invert_motor", false);
	double kP = utils::getParameter<double>(nodeHandle, "kP", 1.0);
	double kI = utils::getParameter<double>(nodeHandle, "kI", 0.0);
	double kD = utils::getParameter<double>(nodeHandle, "kD", 0.0);
	double kF = utils::getParameter<double>(nodeHandle, "kF", 0.0);
	int publishingDelay = utils::getParameter<int>(nodeHandle, "publishing_delay", 0);
	op_mode = utils::getParameter<int>(nodeHandle, "op_mode", 0);
	printData = utils::getParameter<bool>(nodeHandle, "print_data", false);
	resetString = utils::getParameter<std::string>(nodeHandle, "reset_topic", "1");

	// ctre::phoenix::platform::can::SetCANInterface(can_interface.c_str());

	int kTimeoutMs=30;
	int kPIDLoopIdx=0;
	talonSRX=new TalonSRX(motorNumber);
	RCLCPP_INFO(nodeHandle->get_logger(),"created talon instance");

	talonSRX->SetInverted(invertMotor);
	talonSRX->SelectProfileSlot(0,0);
	talonSRX->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, kTimeoutMs);
	talonSRX->SetSensorPhase(true);
	talonSRX->ConfigClosedloopRamp(2);
	
	talonSRX->Config_kF(kPIDLoopIdx, kF, kTimeoutMs);
	talonSRX->Config_kP(kPIDLoopIdx, kP, kTimeoutMs);
	talonSRX->Config_kI(kPIDLoopIdx, kI, kTimeoutMs);
	talonSRX->Config_kD(kPIDLoopIdx, kD, kTimeoutMs);
	talonSRX->ConfigAllowableClosedloopError(kPIDLoopIdx,0,kTimeoutMs);

	talonSRX->Set(ControlMode::PercentOutput, 0);
	talonSRX->Set(ControlMode::Position, 500);

	RCLCPP_INFO(nodeHandle->get_logger(),"configured talon");

	messages::msg::TalonStatus talonStatus;
	auto talonStatusPublisher=nodeHandle->create_publisher<messages::msg::TalonStatus>(infoTopic.c_str(),1);
	auto potentiometerPublisher=nodeHandle->create_publisher<std_msgs::msg::Int32>(potentiometerTopic.c_str(),1);
	auto speedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32>(speedTopic.c_str(),1,speedCallback);
	auto positionSubscriber=nodeHandle->create_subscription<std_msgs::msg::Int32>(positionTopic.c_str(),1,positionCallback);
	resetPublisher=nodeHandle->create_publisher<std_msgs::msg::String>("reset_topic",1);

	auto stopSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP",1,stopCallback);
	auto goSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("GO",1,goCallback);
	auto commHeartbeatSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("comm_heartbeat",1,commHeartbeatCallback);
	auto logicHeartbeatSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("logic_heartbeat",1,logicHeartbeatCallback);
	auto keySubscriber= nodeHandle->create_subscription<messages::msg::KeyState>("key",1,keyCallback);
	
	RCLCPP_INFO(nodeHandle->get_logger(),"set subscribers");

	rclcpp::Rate rate(100);
	auto start = std::chrono::high_resolution_clock::now();
    commPrevious = start;
    logicPrevious = start;
	float maxCurrent = 0.0;
	double busVoltage = 0.0;

	while(rclcpp::ok()){
        // IMPORTANT: Must feed enable to allow motor output calculation in Sim
		ctre::phoenix::unmanaged::FeedEnable(100);
        
        // --- UPDATE SIMULATION PHYSICS ---
        if(GO) updateSimPhysics();

		auto finish = std::chrono::high_resolution_clock::now();

		if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > publishingDelay){

			int deviceID=talonSRX->GetDeviceID();
			busVoltage=talonSRX->GetBusVoltage();
			double outputCurrent=talonSRX->GetOutputCurrent();
			double motorOutputPercent=talonSRX->GetMotorOutputPercent();
			double temperature=talonSRX->GetTemperature();				
			int sensorPosition0=talonSRX->GetSelectedSensorPosition(0);
			double sensorVelocity0=talonSRX->GetSelectedSensorVelocity(0);
			int closedLoopError0=talonSRX->GetClosedLoopError(0);
			double integralAccumulator0=talonSRX->GetIntegralAccumulator(0);
			double errorDerivative0=talonSRX->GetErrorDerivative(0);
		
			talonStatus.device_id=deviceID;	
			talonStatus.bus_voltage=busVoltage;
			talonStatus.output_current=outputCurrent;
			talonStatus.output_percent=motorOutputPercent;
			talonStatus.temperature=temperature;
			talonStatus.sensor_position=sensorPosition0;
			talonStatus.sensor_velocity=sensorVelocity0;
			talonStatus.closed_loop_error=closedLoopError0;
			talonStatus.integral_accumulator=integralAccumulator0;
			talonStatus.error_derivative=errorDerivative0;
			talonStatus.temp_disable = TEMP_DISABLE;
			if(outputCurrent > maxCurrent){
				maxCurrent = outputCurrent;
			}
			talonStatus.max_current = maxCurrent;
			talonStatusPublisher->publish(talonStatus);
			checkTemperature(temperature);
        	start = std::chrono::high_resolution_clock::now();
		}

		if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-commPrevious).count() > 100 || TEMP_DISABLE
		||	std::chrono::duration_cast<std::chrono::milliseconds>(finish-logicPrevious).count() > 100 ){
            // Watchdog Logic
			if(TEMP_DISABLE && printData) RCLCPP_INFO(nodeHandle->get_logger(),"Temp Disable");
			
			talonSRX->Set(ControlMode::PercentOutput, 0.0);
			GO = false;
		}
		rate.sleep();
		rclcpp::spin_some(nodeHandle);
	}
}