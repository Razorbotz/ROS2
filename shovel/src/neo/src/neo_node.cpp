#include "CANSparkMax.hpp"

#include <rclcpp/rclcpp.hpp>
//#include <rclcpp/console.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/empty.hpp>
#include "messages/msg/neo_status.hpp"
#include "utils/utils.hpp"

rclcpp::Node::SharedPtr nodeHandle;
bool GO = false;

CANSparkMax* canSparkMax;
float currentSpeed = 0.0;

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
    currentSpeed = 0.0;
    canSparkMax->set_duty_cycle(currentSpeed, 0);
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


/** @brief Speed Callback Function
 * 
 * Callback function triggered when the node receives
 * a topic with the topic name of neo_speed.  This 
 * function takes the data from the topic and sets 
 * the motor to the speed specified.
 * @param speed
 * @return void
 * */
void speedCallback(const std_msgs::msg::Float32::SharedPtr speed){
	RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
	//std::cout << "---------->>>  " << speed->data << std::endl;
    if(GO){
        currentSpeed = speed->data;
	      canSparkMax->set_duty_cycle(speed->data, 0);
    }
}


int main(int argc,char** argv){
	rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("neo");

	RCLCPP_INFO(nodeHandle->get_logger(),"Starting neo");

    int motorNumber = utils::getParameter<int>("motor_number", 1);
	canSparkMax = new CANSparkMax("can0", motorNumber);

    std::string infoTopic = utils::getParameter<std::string>("info_topic", "unset");
    std::string speedTopic = utils::getParameter<std::string>("speed_topic", "unset");

    messages::msg::NeoStatus neoStatus;
    auto neoStatusPublisher = nodeHandle->create_publisher<messages::msg::NeoStatus>(infoTopic.c_str(),1);
    auto speedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32>(speedTopic.c_str(),1,speedCallback);

    auto stopSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP",1,stopCallback);
	auto goSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("GO",1,goCallback);
    
    rclcpp::Rate rate(20);
    auto start = std::chrono::high_resolution_clock::now();

    while(rclcpp::ok()){
        auto finish = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() > 250000000){
            neoStatus.speed = currentSpeed;
            neoStatusPublisher->publish(neoStatus);
			//RCLCPP_INFO(nodeHandle->get_logger(), "Neo motor position: %d", canSparkMax->get_motor_position());
        }
        rate.sleep();
        rclcpp::spin_some(nodeHandle);
        canSparkMax->send_heartbeat();
    }
}