cmake_minimum_required(VERSION 3.5)
project(realsense)

# C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)cmake_minimum_required(VERSION 3.5)
project(realsense_obstacle_detector)

# C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(obstacle_detector src/obstacle_detector.cpp)
ament_target_dependencies(obstacle_detector
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  geometry_msgs
)

target_link_libraries(obstacle_detector
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  realsense2
)

install(TARGETS
  obstacle_detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(realsense src/realsense_node.cpp)
ament_target_dependencies(realsense
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  geometry_msgs
)

target_link_libraries(realsense
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  realsense2
)

install(TARGETS
  realsense
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

