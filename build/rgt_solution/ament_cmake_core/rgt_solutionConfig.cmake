# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rgt_solution_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rgt_solution_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rgt_solution_FOUND FALSE)
  elseif(NOT rgt_solution_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rgt_solution_FOUND FALSE)
  endif()
  return()
endif()
set(_rgt_solution_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rgt_solution_FIND_QUIETLY)
  message(STATUS "Found rgt_solution: 0.0.0 (${rgt_solution_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rgt_solution' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rgt_solution_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rgt_solution_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rgt_solution_DIR}/${_extra}")
endforeach()
