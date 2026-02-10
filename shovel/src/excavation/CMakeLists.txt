cmake_minimum_required(VERSION 3.5)
project(excavation)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(messages REQUIRED)
find_package(utils REQUIRED)

include_directories(
	include
)

add_executable(${PROJECT_NAME}_node
	src/excavation_node.cpp
)

install(TARGETS ${PROJECT_NAME}_node
	DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Create the test executable
  # "test_excavation" is the name of the test binary
  # "test/test_excavation.cpp" is the file we wrote in the previous step
  ament_add_gtest(test_excavation test/test_excavation.cpp)

  # Allow the test to find "excavation_core.hpp"
  # Change "include" to "src" if your header is in the src folder
  target_include_directories(test_excavation PUBLIC include)
  
  # If you rely on other libraries (like rclcpp), link them here too,
  # though strictly for the logic tests we wrote, you might not need rclcpp.
  # target_link_libraries(test_excavation ${rclcpp_LIBRARIES}) 
endif()

ament_package()
ament_target_dependencies(${PROJECT_NAME}_node
	rclcpp
	std_msgs
	messages
	utils
)
