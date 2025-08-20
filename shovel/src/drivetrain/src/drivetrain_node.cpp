#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <messages/msg/key_state.hpp>

#include "messages/msg/linear_status.hpp"
#include "messages/msg/falcon_status.hpp"

rclcpp::Node::SharedPtr nodeHandle;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon10Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon11Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon12Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon13Publisher;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon10UserPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon11UserPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon12UserPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon13UserPublisher;

/** @file
 * @brief Node to control drive train motors
 * 
 */


/*
Intended Behavior:
This node is intended to limit the slipping of the wheels and perform any
more complex logic related to the wheel motors. Currently, the node only 
takes in the right and left speed values and publishes the values in a ROS2
message with the correct names that the individual motors are subscribed to.

*/

// TODO:
// Look at allowing the user to specify which falcons are listening to 
// which speeds
// Identify when wheels are slipping
// Adjust speeds to account for slipping, limit slip
void driveLeftSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    RCLCPP_INFO(nodeHandle->get_logger(),"driveLeftSpeed: %f", speed->data);
    std_msgs::msg::Float32 outSpeed;
    outSpeed.data = speed->data;
    falcon11Publisher->publish(outSpeed);
    falcon13Publisher->publish(outSpeed);
}


void driveRightSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    RCLCPP_INFO(nodeHandle->get_logger(),"driveRightSpeed: %f", speed->data);
    std_msgs::msg::Float32 outSpeed;
    outSpeed.data = speed->data;
    falcon10Publisher->publish(outSpeed);
    falcon12Publisher->publish(outSpeed);
}


void userLeftSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    RCLCPP_INFO(nodeHandle->get_logger(),"userLeftSpeed: %f", speed->data);
    std_msgs::msg::Float32 outSpeed;
    outSpeed.data = speed->data;
    falcon11UserPublisher->publish(outSpeed);
    falcon13UserPublisher->publish(outSpeed);
}


void userRightSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    RCLCPP_INFO(nodeHandle->get_logger(),"userRightSpeed: %f", speed->data);
    std_msgs::msg::Float32 outSpeed;
    outSpeed.data = speed->data;
    falcon10UserPublisher->publish(outSpeed);
    falcon12UserPublisher->publish(outSpeed);
}


void falcon1Callback(const messages::msg::FalconStatus::SharedPtr speed){

}


void falcon2Callback(const messages::msg::FalconStatus::SharedPtr speed){

}


void falcon3Callback(const messages::msg::FalconStatus::SharedPtr speed){

}


void falcon4Callback(const messages::msg::FalconStatus::SharedPtr speed){

}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("drivetrain");

    auto driveLeftSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("drive_left_speed",1,driveLeftSpeedCallback);
    auto driveRightSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("drive_right_speed",1,driveRightSpeedCallback);
    auto userLeftSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("user_left_speed",1,userLeftSpeedCallback);
    auto userRightSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("user_right_speed",1,userRightSpeedCallback);

    auto falcon1Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>("talon_10_info",1,falcon1Callback);
    auto falcon2Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>("talon_11_info",1,falcon2Callback);
    auto falcon3Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>("talon_12_info",1,falcon3Callback);
    auto falcon4Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>("talon_13_info",1,falcon4Callback);

    falcon10Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_10_speed",1);
    falcon11Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_11_speed",1);
    falcon12Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_12_speed",1);
    falcon13Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_13_speed",1);

    falcon10UserPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_10_user_speed",1);
    falcon11UserPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_11_user_speed",1);
    falcon12UserPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_12_user_speed",1);
    falcon13UserPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_13_user_speed",1);

    auto start = std::chrono::high_resolution_clock::now();
    auto finish = std::chrono::high_resolution_clock::now();
    rclcpp::Rate rate(30);
    while(rclcpp::ok()){
        finish = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > 33){
            start = std::chrono::high_resolution_clock::now();
        }
        rate.sleep();
        rclcpp:spin_some(nodeHandle);
    }
}
