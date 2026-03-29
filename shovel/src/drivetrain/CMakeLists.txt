cmake_minimum_required(VERSION 3.5)
project(drivetrain)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(messages REQUIRED)
find_package(utils REQUIRED)
find_package(nav_msgs)
find_package(geometry_msgs)
find_package(tf2_ros)
find_package(tf2)

include_directories(
	include
)

add_executable(${PROJECT_NAME}_node
	src/drivetrain_node.cpp
)

install(TARGETS ${PROJECT_NAME}_node
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
ament_target_dependencies(${PROJECT_NAME}_node
	rclcpp
	std_msgs
	messages
	utils
	nav_msgs
	geometry_msgs
	tf2_ros
	tf2
)
