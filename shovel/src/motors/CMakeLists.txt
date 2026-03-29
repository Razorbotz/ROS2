cmake_minimum_required(VERSION 3.8)
project(motors)

# Default to C++20 (required by Phoenix 6 2026+)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
#  Required dependencies
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(messages REQUIRED)

# utils — adjust if your utils package has a different name
find_package(utils REQUIRED)

# =============================================================================
#  Optional hardware dependencies (not needed for simulation-only builds)
# =============================================================================

# Phoenix 5 (Falcon 500 / TalonSRX via libCTRE_Phoenix*)
# On your system: headers in /usr/local/include, static libs in /usr/local/lib
set(HAS_PHOENIX5 FALSE)

# Headers
find_path(CTRE_PHOENIX5_INCLUDE NAMES ctre/Phoenix.h
          HINTS /usr/local/include /usr/include /opt/ctre/include)

# Libraries — CTRE ships static .a files on Linux
find_library(CTRE_PHOENIX_LIB
             NAMES CTRE_Phoenix CTRE_Phoenix5
             HINTS /usr/local/lib /usr/lib /opt/ctre/lib)
find_library(CTRE_PHOENIXCCI_LIB
             NAMES CTRE_PhoenixCCI CTRE_Phoenix5CCI
             HINTS /usr/local/lib /usr/lib /opt/ctre/lib)
find_library(CTRE_PHOENIXTOOLS_LIB
             NAMES CTRE_PhoenixCore CTRE_PhoenixTools
             HINTS /usr/local/lib /usr/lib /opt/ctre/lib)
find_library(CTRE_PHOENIXCANUTILS_LIB
             NAMES CTRE_PhoenixCanutils
             HINTS /usr/local/lib /usr/lib /opt/ctre/lib)
find_library(CTRE_PHOENIXDIAG_LIB
             NAMES CTRE_PhoenixDiagnostics
             HINTS /usr/local/lib /usr/lib /opt/ctre/lib)
find_library(CTRE_PHOENIXPLATFORM_LIB
             NAMES CTRE_PhoenixPlatform_socketcan CTRE_PhoenixPlatformLinuxSocketCan
             HINTS /usr/local/lib /usr/lib /opt/ctre/lib)

if(CTRE_PHOENIX5_INCLUDE AND CTRE_PHOENIX_LIB)
  set(HAS_PHOENIX5 TRUE)
  set(PHOENIX5_LIBRARIES ${CTRE_PHOENIX_LIB})
  foreach(OPTLIB
      CTRE_PHOENIXCCI_LIB
      CTRE_PHOENIXTOOLS_LIB
      CTRE_PHOENIXCANUTILS_LIB
      CTRE_PHOENIXDIAG_LIB
      CTRE_PHOENIXPLATFORM_LIB)
    if(${OPTLIB})
      list(APPEND PHOENIX5_LIBRARIES ${${OPTLIB}})
    endif()
  endforeach()
  set(PHOENIX5_INCLUDE_DIRS ${CTRE_PHOENIX5_INCLUDE})
  message(STATUS "Phoenix 5 FOUND: ${PHOENIX5_LIBRARIES}")
else()
  message(STATUS "Phoenix 5 NOT found — hardware backends disabled for Falcon/TalonSRX")
  if(NOT CTRE_PHOENIX5_INCLUDE)
    message(STATUS "  (headers not found: ctre/Phoenix.h)")
  endif()
  if(NOT CTRE_PHOENIX_LIB)
    message(STATUS "  (library not found: CTRE_Phoenix)")
  endif()
endif()

# Phoenix 6 (Kraken x60)
# apt install phoenix6 puts:
#   headers in /usr/include/phoenix6/
#   libs in    /usr/lib/phoenix6/
#   cmake in   /usr/lib/phoenix6/cmake/
set(HAS_PHOENIX6 FALSE)

# Try the shipped CMake config first
find_package(phoenix6 QUIET
             HINTS /usr/lib/phoenix6/cmake)

if(phoenix6_FOUND)
  set(HAS_PHOENIX6 TRUE)
  message(STATUS "Phoenix 6 FOUND via cmake config")
else()
  # Fallback to manual detection
  find_path(CTRE_PHOENIX6_INCLUDE NAMES ctre/phoenix6/TalonFX.hpp
            HINTS /usr/include/phoenix6 /usr/include
                  /usr/local/include /opt/ctre/include)

  find_library(CTRE_PHOENIX6_LIB
               NAMES CTRE_Phoenix6
               HINTS /usr/lib/phoenix6 /usr/lib /usr/local/lib /opt/ctre/lib)

  if(CTRE_PHOENIX6_LIB AND CTRE_PHOENIX6_INCLUDE)
    set(HAS_PHOENIX6 TRUE)
    set(PHOENIX6_LIBRARIES ${CTRE_PHOENIX6_LIB})
    set(PHOENIX6_INCLUDE_DIRS ${CTRE_PHOENIX6_INCLUDE})
    message(STATUS "Phoenix 6 FOUND (manual): ${PHOENIX6_LIBRARIES}")
  else()
    message(STATUS "Phoenix 6 NOT found — hardware backend disabled for Kraken")
    if(NOT CTRE_PHOENIX6_INCLUDE)
      message(STATUS "  (headers not found: ctre/phoenix6/TalonFX.hpp)")
    endif()
    if(NOT CTRE_PHOENIX6_LIB)
      message(STATUS "  (library not found: CTRE_Phoenix6)")
    endif()
  endif()
endif()

# =============================================================================
#  Common ament dependencies list
# =============================================================================
set(COMMON_DEPS
  rclcpp
  std_msgs
  messages
  utils
)

set(DRIVE_DEPS
  ${COMMON_DEPS}
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
)

set(TALON_DEPS
  ${COMMON_DEPS}
  sensor_msgs
)

# =============================================================================
#  drive_motor_node — unified Falcon 500 / Kraken x60 / simulation
# =============================================================================
add_executable(drive_motor_node src/drive_motor_node.cpp)

target_include_directories(drive_motor_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(drive_motor_node ${DRIVE_DEPS})

# Conditionally compile hardware backends
if(HAS_PHOENIX5)
  target_compile_definitions(drive_motor_node PRIVATE HAS_PHOENIX5)
  target_include_directories(drive_motor_node PRIVATE ${PHOENIX5_INCLUDE_DIRS})
  target_link_libraries(drive_motor_node ${PHOENIX5_LIBRARIES})
endif()

if(HAS_PHOENIX6)
  target_compile_definitions(drive_motor_node PRIVATE HAS_PHOENIX6)
  if(phoenix6_FOUND)
    # Found via cmake config — use imported targets
    target_link_libraries(drive_motor_node phoenix6)
  else()
    # Found manually — use explicit paths
    target_include_directories(drive_motor_node PRIVATE ${PHOENIX6_INCLUDE_DIRS})
    target_link_libraries(drive_motor_node ${PHOENIX6_LIBRARIES})
  endif()
endif()

# =============================================================================
#  talon_node — unified TalonSRX actuator / simulation
# =============================================================================
add_executable(talon_node src/talon_node.cpp)

target_include_directories(talon_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(talon_node ${TALON_DEPS})

if(HAS_PHOENIX5)
  target_compile_definitions(talon_node PRIVATE HAS_PHOENIX5)
  target_include_directories(talon_node PRIVATE ${PHOENIX5_INCLUDE_DIRS})
  target_link_libraries(talon_node ${PHOENIX5_LIBRARIES})
endif()

# =============================================================================
#  Install
# =============================================================================
install(TARGETS
  drive_motor_node
  talon_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# =============================================================================
#  Testing
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()