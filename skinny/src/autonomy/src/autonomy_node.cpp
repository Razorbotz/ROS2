#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "messages/msg/zed_position.hpp"

/** @file
* @brief Node handling Autonomy for robot
*
* This node recieves information about zedPosition, 
* and publishes speedLeft and SpeedRight information.
*
* The topics that are being subscribed to are as follows:
* \li \b ZedPosition
*
* The topics that are being published are as follows:
* \li \b speedLeft
* \li \b speedRight
*
*
**/

rclcpp::Node::SharedPtr nodeHandle;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveLeftSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveRightSpeedPublisher;

/** @brief Callback function for zedPosition
*
* This function is called when the node recieves a topic 
* with the name zedPosition. This function sets the 
* speedLeft and speedRight data depending on aruco_visible and z 
* values. It then logs and publishes speedLeft and speedRight.
*
* The first statement checks if the marker is visible and the z position is greater than 1 meter away
* if so, the left and right wheels move forward.
* The second statement stops the wheels
* The third statement rotates the wheels in opposite directions until the marker is visible
* 
* @param zedPosition \see ZedPosition.msg
* @return void
* */


void positionCallback(const messages::msg::ZedPosition::SharedPtr zedPosition){
	std_msgs::msg::Float32 speedLeft;
	std_msgs::msg::Float32 speedRight;
	if(zedPosition->aruco_visible && zedPosition->z > 1){
		speedLeft.data = 0.2;
		speedRight.data = 0.2;
	}
	else if(zedPosition->aruco_visible && zedPosition-> z < 2.5 && zedPosition->z > 0){
		speedLeft.data = 0.0;
		speedRight.data = 0.0;
	}
	else if(!zedPosition->aruco_visible){
		speedLeft.data = 0.2;
		speedRight.data = -0.2;
	}

	RCLCPP_INFO(nodeHandle->get_logger(), "speed left=%f right=%f", speedLeft.data, speedRight.data);
	driveLeftSpeedPublisher->publish(speedLeft);
	driveRightSpeedPublisher->publish(speedRight);
}

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	nodeHandle = rclcpp::Node::make_shared("autonomy");
	
	auto zedPositionSubscriber = nodeHandle->create_subscription<messages::msg::ZedPosition>("zed_position", 1, positionCallback);
	
	driveLeftSpeedPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("drive_left_speed", 1);
	driveRightSpeedPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("drive_right_speed", 1);

	rclcpp::Rate rate(10);
	while(rclcpp::ok()){
		rclcpp::spin_some(nodeHandle);
	}
}
