cmake_minimum_required(VERSION 3.5)
project(falcon)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(messages REQUIRED)
find_package(common_funcs REQUIRED)

include_directories(
	include
	common
)

add_executable(${PROJECT_NAME}_node
	src/falcon_node.cpp
	../common_funcs/src/CommonFuncs.cpp
)

target_link_libraries(${PROJECT_NAME}_node
	libCTRE_Phoenix.a
	libCTRE_PhoenixCCI.a
	libCTRE_PhoenixPlatform_socketcan.a
	libCTRE_PhoenixCanutils.a
	libCTRE_PhoenixCore.a
	libCTRE_PhoenixDiagnostics.a
	libpthread.so
)

install(TARGETS ${PROJECT_NAME}_node
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
ament_target_dependencies(${PROJECT_NAME}_node
	rclcpp
	std_msgs
	messages
	common_funcs
)
