cmake_minimum_required(VERSION 3.5)
project(logic)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
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
	src/logic_node.cpp
	src/Automation.cpp
	src/Automation1.cpp
	src/Automation2.cpp
	src/Automation3.cpp
	src/search.cpp
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

