cmake_minimum_required(VERSION 3.5)
project(nano_communication)

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
find_package(utils REQUIRED)
find_package(ZLIB REQUIRED)

include_directories(
        include
        ../../../shovel/src/communication/include
)

add_executable(${PROJECT_NAME}_node
	../../../shovel/src/communication/src/BinaryMessage.cpp
        ../../../shovel/src/communication/src/Heartbeat.cpp
        ../../../shovel/src/communication/src/AegisBase.cpp
        ../../../shovel/src/communication/src/AegisNanoController.cpp
        ../../../shovel/src/communication/src/CANHeartbeat.cpp
        src/communication_node.cpp
)

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_node ${ZLIB_LIBRARIES})

ament_package()
ament_target_dependencies(${PROJECT_NAME}_node 
	rclcpp 
	std_msgs 
	messages
        utils
        
)

