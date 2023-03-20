#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

rclcpp::Node::SharedPtr nodeHandle;

int main(int argc, char **argv){
        rclcpp::init(argc, argv);
        nodeHandle = rclcpp::Node::make_shared("test");
        auto potentiometerPublisher = nodeHandle->create_publisher<std_msgs::msg::Int16MultiArray>("potentiometer_data",1);
        auto shoulderPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("shoulder_speed",1);
        
        std_msgs::msg::Float32 speed1;
        speed1.data = 0.5;  
        shoulderPublisher->publish(speed1);
        
        rclcpp::Rate rate(10);
        
		std_msgs::msg::Int16MultiArray pot;
		int count = 0;
        while(rclcpp::ok()){
			rclcpp::spin_some(nodeHandle);
			pot.data = [count, 450, 30];
			potentiometerPublisher->publish(pot);
			count += 10;
			rate.sleep();
        }
}