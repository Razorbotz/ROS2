#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <messages/msg/key_state.hpp>

#include "messages/msg/linear_status.hpp"
#include "messages/msg/falcon_status.hpp"
#include "messages/msg/drivetrain_status.hpp"
#include "utils/utils.hpp"

rclcpp::Node::SharedPtr nodeHandle;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon10Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon11Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon12Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon13Publisher;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon10UserPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon11UserPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon12UserPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falcon13UserPublisher;
std::shared_ptr<rclcpp::Publisher<messages::msg::DrivetrainStatus_<std::allocator<void> >, std::allocator<void> > > drivetrainStatusPublisher;

double wheelDiameter = .2;
double wheelCircum = wheelDiameter * M_PI;
bool printData = false;
double gearReduction = 100.0;
const double SENSOR_UNITS_PER_ROTATION = 2048.0;
const double CONVERT_TO_SEC = 600.0;

double falcon1RawVelocity, falcon2RawVelocity, falcon3RawVelocity, falcon4RawVelocity;
double falcon1RPM, falcon2RPM, falcon3RPM, falcon4RPM;
double falcon1GroundSpeed, falcon2GroundSpeed, falcon3GroundSpeed, falcon4GroundSpeed;


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


// Testing code to translate velocity into rpm
// Falcon encoder has 2048 ticks / rev and reads every 100ms
// To translate from vel / 100 ms to rpm, multiply by 600
// Also going to try to estimate ground speed
void falcon1Callback(const messages::msg::FalconStatus::SharedPtr speed){
    falcon1RawVelocity = speed->sensor_velocity;
    falcon1RPM = falcon1RawVelocity * CONVERT_TO_SEC / (SENSOR_UNITS_PER_ROTATION * gearReduction);
    falcon1GroundSpeed = falcon1RPM / 60.0 * wheelCircum;

    if(printData){
        RCLCPP_INFO(nodeHandle->get_logger(), "Falcon 1 raw velocity: %f", falcon1RawVelocity);
        RCLCPP_INFO(nodeHandle->get_logger(), "Falcon 1 rpm: %f", falcon1RPM);
        RCLCPP_INFO(nodeHandle->get_logger(), "Falcon 1 groundSpeed: %f", falcon1GroundSpeed);
    }
}


void falcon2Callback(const messages::msg::FalconStatus::SharedPtr speed){
    falcon2RawVelocity = speed->sensor_velocity;
    falcon2RPM = falcon2RawVelocity * CONVERT_TO_SEC / (SENSOR_UNITS_PER_ROTATION * gearReduction);
    falcon2GroundSpeed = falcon2RPM / 60.0 * wheelCircum;

    if(printData){
        RCLCPP_INFO(nodeHandle->get_logger(), "Falcon 2 raw velocity: %f", falcon2RawVelocity);
        RCLCPP_INFO(nodeHandle->get_logger(), "Falcon 2 rpm: %f", falcon2RPM);
        RCLCPP_INFO(nodeHandle->get_logger(), "Falcon 2 groundSpeed: %f", falcon2GroundSpeed);
    }
}


void falcon3Callback(const messages::msg::FalconStatus::SharedPtr speed){
    falcon3RawVelocity = speed->sensor_velocity;
    falcon3RPM = falcon3RawVelocity * CONVERT_TO_SEC / (SENSOR_UNITS_PER_ROTATION * gearReduction);
    falcon3GroundSpeed = falcon3RPM / 60.0 * wheelCircum;

    if(printData){
        RCLCPP_INFO(nodeHandle->get_logger(), "Falcon 3 raw velocity: %f", falcon3RawVelocity);
        RCLCPP_INFO(nodeHandle->get_logger(), "Falcon 3 rpm: %f", falcon3RPM);
        RCLCPP_INFO(nodeHandle->get_logger(), "Falcon 3 groundSpeed: %f", falcon3GroundSpeed);
    }
}


void falcon4Callback(const messages::msg::FalconStatus::SharedPtr speed){
    falcon4RawVelocity = speed->sensor_velocity;
    falcon4RPM = falcon4RawVelocity * CONVERT_TO_SEC / (SENSOR_UNITS_PER_ROTATION * gearReduction);
    falcon4GroundSpeed = falcon4RPM / 60.0 * wheelCircum;

    if(printData){
        RCLCPP_INFO(nodeHandle->get_logger(), "Falcon 4 raw velocity: %f", falcon4RawVelocity);
        RCLCPP_INFO(nodeHandle->get_logger(), "Falcon 4 rpm: %f", falcon4RPM);
        RCLCPP_INFO(nodeHandle->get_logger(), "Falcon 4 groundSpeed: %f", falcon4GroundSpeed);
    }
}


void publishStatus(){
    messages::msg::DrivetrainStatus drivetrainStatus;
    drivetrainStatus.falcon1_velocity = falcon1RawVelocity;
    drivetrainStatus.falcon1_rpm = falcon1RPM;
    drivetrainStatus.falcon1_ground_speed = falcon1GroundSpeed;
    drivetrainStatus.falcon2_velocity = falcon2RawVelocity;
    drivetrainStatus.falcon2_rpm = falcon2RPM;
    drivetrainStatus.falcon2_ground_speed = falcon2GroundSpeed;
    drivetrainStatus.falcon3_velocity = falcon3RawVelocity;
    drivetrainStatus.falcon3_rpm = falcon3RPM;
    drivetrainStatus.falcon3_ground_speed = falcon3GroundSpeed;
    drivetrainStatus.falcon4_velocity = falcon4RawVelocity;
    drivetrainStatus.falcon4_rpm = falcon4RPM;
    drivetrainStatus.falcon4_ground_speed = falcon4GroundSpeed;
    drivetrainStatusPublisher->publish(drivetrainStatus);
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("drivetrain");
    printData  = utils::getParameter<bool>(nodeHandle, "print_data", false);

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

    falcon10UserPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_10_speed",1);
    falcon11UserPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_11_speed",1);
    falcon12UserPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_12_speed",1);
    falcon13UserPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_13_speed",1);

    drivetrainStatusPublisher = nodeHandle->create_publisher<messages::msg::DrivetrainStatus>("drivetrain_status",1);

    auto start = std::chrono::high_resolution_clock::now();
    auto finish = std::chrono::high_resolution_clock::now();
    int counter = 0;
    rclcpp::Rate rate(60);
    while(rclcpp::ok()){
        finish = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > 33){
            start = std::chrono::high_resolution_clock::now();
        }
        publishStatus();
        rate.sleep();
        rclcpp:spin_some(nodeHandle);
    }
}
