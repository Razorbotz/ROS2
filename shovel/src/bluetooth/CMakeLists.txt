cmake_minimum_required(VERSION 3.8)
project(bluetooth)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/bt_server_node.py
  scripts/bt_client_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()