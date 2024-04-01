# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_power_distribution_panel_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED power_distribution_panel_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(power_distribution_panel_FOUND FALSE)
  elseif(NOT power_distribution_panel_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(power_distribution_panel_FOUND FALSE)
  endif()
  return()
endif()
set(_power_distribution_panel_CONFIG_INCLUDED TRUE)

# output package information
if(NOT power_distribution_panel_FIND_QUIETLY)
  message(STATUS "Found power_distribution_panel: 1.0.0 (${power_distribution_panel_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'power_distribution_panel' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${power_distribution_panel_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(power_distribution_panel_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${power_distribution_panel_DIR}/${_extra}")
endforeach()
