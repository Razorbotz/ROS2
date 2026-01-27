cmake_minimum_required(VERSION 3.8)
project(control_gui)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)
find_package(CURL REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED) # For pthread

pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)
pkg_check_modules(CAIRO REQUIRED cairo)
pkg_check_modules(WEBKITGTK REQUIRED webkit2gtk-4.0)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
    libavcodec
    libavformat
    libavutil
    libswscale
)

add_definitions(${GTKMM_CFLAGS_OTHER})

# Define the executable with all its source files
add_executable(control_gui
  src/control.cpp
  src/InfoFrame.cpp
  src/InfoItem.cpp
  src/BinaryMessage.cpp
  src/Speedometer.cpp
  src/ConfigDefinitions.cpp
  src/ConfigEditorWindow.cpp
  src/NetworkHandler.cpp
)

# Set the include directories using the variables from find_package
target_include_directories(control_gui PUBLIC
  # This is for your local headers (e.g., BinaryMessage.hpp)
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  
  # System Includes from find_package and pkg_check_modules
  ${GTKMM_INCLUDE_DIRS}
  ${CAIRO_INCLUDE_DIRS}
  ${WEBKITGTK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${ZLIB_INCLUDE_DIRS}
  ${FFMPEG_INCLUDE_DIRS}
)

# Link the ROS2 libraries
ament_target_dependencies(control_gui
  rclcpp
  sensor_msgs
  cv_bridge
  ament_index_cpp
)

# Link all system libraries using variables
target_link_libraries(control_gui
  # System Libs
  ${GTKMM_LIBRARIES}
  ${CAIRO_LIBRARIES}
  ${WEBKITGTK_LIBRARIES}
  ${OpenCV_LIBS}
  ${SDL2_LIBRARIES}
  ${ZLIB_LIBRARIES}
  Threads::Threads
  CURL::libcurl
  PkgConfig::FFMPEG
)

# --- Installation Rules ---

# Install the executable
install(TARGETS control_gui
  DESTINATION lib/${PROJECT_NAME}
)

# Install all public headers (assuming they are in 'include/')
install(DIRECTORY include/
  DESTINATION include
)

# Install the resource files (images, fonts, etc.)
install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

# Install the package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Finalize the package
ament_package()