# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_logic_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED logic_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(logic_FOUND FALSE)
  elseif(NOT logic_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(logic_FOUND FALSE)
  endif()
  return()
endif()
set(_logic_CONFIG_INCLUDED TRUE)

# output package information
if(NOT logic_FIND_QUIETLY)
  message(STATUS "Found logic: 0.0.0 (${logic_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'logic' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${logic_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(logic_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${logic_DIR}/${_extra}")
endforeach()
