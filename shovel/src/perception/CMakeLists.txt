cmake_minimum_required(VERSION 3.5)
project(perception)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17) # Changed to 17, recommended for newer PCL/ROS 2 Humble
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(messages REQUIRED)
find_package(utils REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/perception_node.cpp
)

# Link PCL libraries
target_link_libraries(${PROJECT_NAME}_node 
    ${ZLIB_LIBRARIES}
    ${PCL_LIBRARIES}
)

# Add ROS 2 packages to ament
ament_target_dependencies(${PROJECT_NAME}_node 
    rclcpp 
    std_msgs 
    messages
    utils
    sensor_msgs
    vision_msgs
    pcl_conversions
    visualization_msgs
    tf2_ros
    tf2_sensor_msgs
    geometry_msgs
)

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()