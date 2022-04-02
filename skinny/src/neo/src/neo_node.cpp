#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>


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


#include <rclcpp/rclcpp.hpp>
//#include <rclcpp/console.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>

#include <rev/CANSparkMax.h>

/** @file
 * @brief Node controlling one Neo motor 
 * 
 * This node receives information published by the logic node,
 * then transforms the data received into movement by the motor
 * controlled by the Neo instance.  The topics that the node
 * subscribes to are as follows:
 * \li \b speed_topic
 * \li \b STOP
 * \li \b GO
 * 
 * The \b speed_topic topic is either \b drive_left_speed
 * or \b drive_right_speed as defined in the parameters set in
 * the launch file.  To read more about the logic node or the
 * launch file
 * \see logic_node.cpp
 * \see launch.py
 * 
 * The topics being published are as follows:
 * \li \b info_topic
 * 
 * This string has the general form neo_{motorNumber}_info and
 * is defined by the user in the launch file.  To read more about
 * the launch file,
 * \see launch.py
 * 
 * */

rclcpp::Node::SharedPtr nodeHandle;
//bool GO=false;
bool GO = true;

/** @brief STOP Callback
 * 
 * Callback function triggered when the node receives
 * a topic with the topic name of STOP.  This function
 * sets a boolean value GO to false, which prevents the
 * robot from moving.
 * @param empty
 * @return void
 * */
void stopCallback(std_msgs::msg::Empty::SharedPtr empty){
	RCLCPP_INFO(nodeHandle->get_logger(),"STOP");
	GO=false;
} 

/** @brief GO Callback
 * 
 * Callback function triggered when the node receives
 * a topic with the topic name of GO.  This function
 * sets a boolean value GO to true, which allows the
 * robot to drive.
 * @param empty
 * @return void
 * */
void goCallback(std_msgs::msg::Empty::SharedPtr empty){
	RCLCPP_INFO(nodeHandle->get_logger(),"GO");
	GO=true;
}

bool useVelocity=false;
int velocityMultiplier=0;
int testSpeed=0;

//init pidController
rev::CANPIDController pidController_front;
rev::CANPIDController pidController_back;


/** @brief Speed Callback Function
 * 
 * Callback function triggered when the node receives
 * a topic with the topic name of drive_left_speed or
 * drive_right_speed.  This function takes the data
 * from the topic and sets the motor to the speed
 * specified.
 * @param speed
 * @return void
 * */
void speedCallback(const std_msgs::msg::Float32::SharedPtr speed){
//	RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
	//std::cout << "---------->>>  " << speed->data << std::endl;

	//sets motor at set speed
	if(useVelocity){
		pidController.SetReference(speed->data*velocityMultiplier, rev::ControlType::kVelocity);

	//sets motor at percent times max rpm
	}else{
		//percent passed 
		double percent = speed->data;

		//https://motors.vex.com/other-motors/neo
		//max published rpm (can go higher but don't want to blow up)
		int max_rpm = 5676;
        pidController.SetReference(double(percent*max_rpm), rev::ControlType::kVelocity);
	}
}

/** @brief String parameter function
 * 
 * Function that takes a string as a parameter containing the
 * name of the parameter that is being parsed from the launch
 * file and the initial value of the parameter as inputs, then
 * gets the parameter, casts it as a string, displays the value
 * of the parameter on the command line and the log file, then
 * returns the parsed value of the parameter.
 * @param parametername String of the name of the parameter
 * @param initialValue Initial value of the parameter
 * @return value Value of the parameter
 * */
template <typename T>
T getParameter(std::string parameterName, std::string initialValue){
	nodeHandle->declare_parameter<T>(parameterName, initialValue);
	rclcpp::Parameter param = nodeHandle->get_parameter(parameterName);
	T value = param.as_string();
	std::cout << parameterName << ": " << value << std::endl;
	std::string output = parameterName + ": " + value;
	RCLCPP_INFO(nodeHandle->get_logger(), output);
	return value;
}

/** @brief Parameter function
 * 
 * Function that takes a string as a parameter containing the
 * name of the parameter that is being parsed from the launch
 * file and the initial value of the parameter as inputs, then
 * gets the parameter, casts it as the desired type, displays 
 * the value of the parameter on the command line and the log 
 * file, then returns the parsed value of the parameter.
 * The type parameter uses the following values:
 * 0 - Int
 * 1 - Bool
 * 2 - Double
 * @param parametername String of the name of the parameter
 * @param initialValue Initial value of the parameter
 * @param type Specifies the desired type
 * @return value Value of the parameter
 * */
template <typename T>
T getParameter(std::string parameterName, int initialValue, int type){
	nodeHandle->declare_parameter<T>(parameterName, initialValue);
	rclcpp::Parameter param = nodeHandle->get_parameter(parameterName);
	T value;
	switch(type){
		case 0:
			value = param.as_int();
			break;
		case 1:
			value = param.as_bool();
			break;
		case 2:
			value = param.as_double();
			break;
	}
	std::cout << parameterName << ": " << value << std::endl;
	std::string output = parameterName + ": " + std::to_string(value);
	RCLCPP_INFO(nodeHandle->get_logger(), output);
	return value;
}

//set motor pidController settings
void set_motor_settings(
	double kP,
	double kI,
	double kD,
	double kIz,
	double kFF,
	double kMinOutput,
	double kMaxOutput,
	rev::CANPIDController *pidController
) 
{
	//proportional gain constant
	pidController.SetP(kP);
	// Integral Gain constant
	pidController.SetI(kI);
	//Derivative Gain constant
	pidController.SetD(kD);
	//IZone constant
	pidController.SetIZone(kIz);
	// Feed-forward Gain
	pidController.SetFF(kFF);
	//set reverse power min and forward power max
	pidController.SetOutputRange(kMinOutput, kMaxOutput);
}

int main(int argc, char** argv){
	rclcpp::init(argc,argv);
	nodeHandle = rclcpp::Node::make_shared("neo");
	RCLCPP_INFO(nodeHandle->get_logger(),"Starting neo");

	//geting params
	int motorNumberFront = getParameter<int>("motor_number_front", 1, 0);
	int motorNumberBack = getParameter<int>("motor_number_back", 1, 0);

	std::string infoTopic = getParameter<std::string>("info_topic", "unset");
	std::string speedTopic = getParameter<std::string>("speed_topic", "unset");

	bool invertMotor = getParameter<bool>("invert_motor", 0, 1);
	useVelocity = getParameter<bool>("use_velocity", 0, 1);
	velocityMultiplier = getParameter<int>("velocity_multiplier", 0, 0);
	
	testSpeed = getParameter<int>("test_speed", 0, 0);

	//<type>(name_of_var, init_val, type_of_var)

	//type_of_var:
	//0 then int
	//1 then bool
	//2 then double
	double kP = getParameter<double>("kP", 0, 2);
	double kI = getParameter<double>("kI", 0, 2);
	double kD = getParameter<double>("kD", 0, 2);
	double kIz = getParameter<double>("kIz", 0, 2);
	double kFF = getParameter<double>("kFF", 0, 2);
	double kMinOutput = getParameter<double>("kMinOutput", 0, 2);
	double kMaxOutput = getParameter<double>("kMaxOutput", 0, 2);


	//front motor
	rev::CANSparkMax sparkMax_front{motorNumberFront, rev::CANSparkMax::MotorType::kBrushless};
	sparkMax_front.RestoreFactoryDefaults();

	//pidController setup
	pidController_front  = sparkMax_front.GetPIDController();

	//setup pidController
	set_motor_settings(
		kP,
		kI,
		kD,
		kIz,
		kFF,
		kMinOutput,
		kMaxOutput,
		&pidController_front
	);

	// //init front encoder (not required for us)
	// rev::CANEncoder encoder_front = sparkMax_front.GetEncoder(rev::CANEncoder::EncoderType::kQuadrature, 4096);
	// encoder.SetInverted(invertMotor);

	// //set controller's feedback device 
	// pidController_front.SetFeedbackDevice(encoder_front);

	//back motor
	rev::CANSparkMax sparkMax_back{motorNumberBack, rev::CANSparkMax::MotorType::kBrushless};
	sparkMax_back.RestoreFactoryDefaults();

	//pidController setup
	pidController_back  = sparkMax_back.GetPIDController();

	//setup pidController
	set_motor_settings(
		kP,
		kI,
		kD,
		kIz,
		kFF,
		kMinOutput,
		kMaxOutput,
		&pidController_back
	);

	//back motor to mirror the front
	sparkMax_back.Follow(CANSparkMax &sparkMax_front, false);
)	

	// //init back encoder (not required for us)
	// rev::CANEncoder encoder_back = sparkMax_back.GetEncoder(rev::CANEncoder::EncoderType::kQuadrature, 4096);
	// encoder.SetInverted(invertMotor);

	// //set controller's feedback device 
	// pidController_back.SetFeedbackDevice(encoder_back);

	auto speedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32(speedTopic.c_str(),1,speedCallback);
	auto stopSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP",1,stopCallback);
	auto goSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty("GO", 1, goCallback);
	RCLCPP_INFO(nodeHandle->get_logger(), "Set subscribers");

	RCLCPP_INFO(nodeHandle->get_logger(),"Created NEO");
	while(rclcpp::ok()){
		rclcpp::spin_some(nodeHandle);
	}
}
