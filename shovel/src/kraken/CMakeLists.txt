cmake_minimum_required(VERSION 3.5)
project(kraken)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(messages REQUIRED)
find_package(utils REQUIRED)
find_package(phoenix6 REQUIRED)

find_library(CTRE_PHOENIX6_LIB 
	NAMES CTRE_Phoenix6 
	PATHS /usr/lib/phoenix6 
	NO_DEFAULT_PATH
)

include_directories(
	include
)

add_executable(${PROJECT_NAME}_node
	src/kraken_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
	${CTRE_PHOENIX6_LIB}
	phoenix6
	pthread
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES
	BUILD_WITH_INSTALL_RPATH TRUE
	INSTALL_RPATH "/usr/lib/phoenix6"
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
)