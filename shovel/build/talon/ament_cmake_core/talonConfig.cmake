# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_talon_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED talon_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(talon_FOUND FALSE)
  elseif(NOT talon_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(talon_FOUND FALSE)
  endif()
  return()
endif()
set(_talon_CONFIG_INCLUDED TRUE)

# output package information
if(NOT talon_FIND_QUIETLY)
  message(STATUS "Found talon: 1.0.0 (${talon_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'talon' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${talon_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(talon_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${talon_DIR}/${_extra}")
endforeach()
