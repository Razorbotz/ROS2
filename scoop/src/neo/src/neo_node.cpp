#include "CANSparkMax.hpp"

#include <rclcpp/rclcpp.hpp>
//#include <rclcpp/console.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/empty.hpp>


rclcpp::Node::SharedPtr nodeHandle;
CANSparkMax canSparkMax("can0", 16);

int main(int argc,char** argv){
	rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("neo");

	RCLCPP_INFO(nodeHandle->get_logger(),"Starting neo");

    canSparkMax.set_duty_cycle(0.02, 0);

    rclcpp::Rate rate(20);

    while(rclcpp::ok()){
        rate.sleep();
        rclcpp::spin_some(nodeHandle);
        canSparkMax.send_heartbeat();
    }
}