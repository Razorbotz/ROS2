CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(video_streaming)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

# ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(messages REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# ZED
find_package(ZED 3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

# FFmpeg (for other parts of your build)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
    libavcodec
    libavformat
    libavutil
    libswscale
)

# Jetson Multimedia API paths
set(MMAPI_INC /usr/src/jetson_multimedia_api/include)
set(MMAPI_COMMON /usr/src/jetson_multimedia_api/samples/common)
set(MMAPI_CLASSES ${MMAPI_COMMON}/classes)

include_directories(
    ${CUDA_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${FFMPEG_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    SYSTEM ${MMAPI_INC} ${MMAPI_COMMON} ${MMAPI_CLASSES}
)

# lib search paths
link_directories(
    ${ZED_LIBRARY_DIR}
    ${OpenCV_LIBRARY_DIRS}
    ${CUDA_LIBRARY_DIRS}
    /usr/lib/aarch64-linux-gnu
    /usr/lib/aarch64-linux-gnu/tegra
)

# Build MMAPI helper classes (this provides NvVideoEncoder symbols!)
add_library(mmapi_common STATIC
    ${MMAPI_CLASSES}/NvElement.cpp
    ${MMAPI_CLASSES}/NvElementProfiler.cpp
    ${MMAPI_CLASSES}/NvLogging.cpp
    ${MMAPI_CLASSES}/NvBuffer.cpp
    ${MMAPI_CLASSES}/NvUtils.cpp
    ${MMAPI_CLASSES}/NvV4l2Element.cpp
    ${MMAPI_CLASSES}/NvV4l2ElementPlane.cpp
    ${MMAPI_CLASSES}/NvVideoEncoder.cpp
)

# runtime path & link flags for NVIDIA encoder libs
target_link_options(mmapi_common PRIVATE
    -Wl,--no-as-needed
    -Wl,-rpath,/usr/lib/aarch64-linux-gnu/tegra
)

target_link_libraries(mmapi_common
    nvbufsurface
    nvbufsurftransform
    nvmm
    v4l2
    nvv4l2
    pthread
    dl
)

# Collect node sources
FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
FILE(GLOB_RECURSE HDR_FILES include/*.hpp)

ADD_EXECUTABLE(${PROJECT_NAME}_node ${HDR_FILES} ${SRC_FILES})
add_definitions(-std=c++11 -O3)

# Link NVIDIA libs FIRST then mmapi, then ROS deps
if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

target_link_options(${PROJECT_NAME}_node PRIVATE
    -Wl,--no-as-needed
    -Wl,-rpath,/usr/lib/aarch64-linux-gnu/tegra
)

target_link_libraries(${PROJECT_NAME}_node
    mmapi_common                     # <-- provides NvVideoEncoder implementation
    ${ZED_LIBS}
    ${OpenCV_LIBRARIES}
    PkgConfig::FFMPEG

    nvbufsurface
    nvbufsurftransform
    nvmm
    v4l2
    nvv4l2

    pthread
    dl
)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    std_msgs
    sensor_msgs
    messages
    cv_bridge
    image_transport
)

install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
