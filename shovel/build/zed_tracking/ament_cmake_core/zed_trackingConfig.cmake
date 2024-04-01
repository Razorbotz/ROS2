# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_zed_tracking_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED zed_tracking_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(zed_tracking_FOUND FALSE)
  elseif(NOT zed_tracking_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(zed_tracking_FOUND FALSE)
  endif()
  return()
endif()
set(_zed_tracking_CONFIG_INCLUDED TRUE)

# output package information
if(NOT zed_tracking_FIND_QUIETLY)
  message(STATUS "Found zed_tracking: 1.0.0 (${zed_tracking_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'zed_tracking' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${zed_tracking_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(zed_tracking_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${zed_tracking_DIR}/${_extra}")
endforeach()
