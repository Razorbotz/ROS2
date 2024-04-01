# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_falcon_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED falcon_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(falcon_FOUND FALSE)
  elseif(NOT falcon_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(falcon_FOUND FALSE)
  endif()
  return()
endif()
set(_falcon_CONFIG_INCLUDED TRUE)

# output package information
if(NOT falcon_FIND_QUIETLY)
  message(STATUS "Found falcon: 1.0.0 (${falcon_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'falcon' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${falcon_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(falcon_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${falcon_DIR}/${_extra}")
endforeach()
