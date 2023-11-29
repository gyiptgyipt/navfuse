# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_navfuse_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED navfuse_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(navfuse_FOUND FALSE)
  elseif(NOT navfuse_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(navfuse_FOUND FALSE)
  endif()
  return()
endif()
set(_navfuse_CONFIG_INCLUDED TRUE)

# output package information
if(NOT navfuse_FIND_QUIETLY)
  message(STATUS "Found navfuse: 0.0.0 (${navfuse_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'navfuse' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${navfuse_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(navfuse_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${navfuse_DIR}/${_extra}")
endforeach()
