cmake_minimum_required(VERSION 3.5)
project(neo)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(messages REQUIRED)


include_directories(
	include
)

add_executable(${PROJECT_NAME}_node 
	src/neo_node.cpp 
)

target_link_libraries(${PROJECT_NAME}_node
	libwpilibc.so
	libwpiutil.so
	libwpimath.so
	libntcore.so
	libwpiHal.so
	libREVLibDriver.so
	libREVLib.so
	
	
)

install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
        std_msgs
        messages
)
