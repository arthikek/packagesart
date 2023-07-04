# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_raspimouse_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED raspimouse_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(raspimouse_FOUND FALSE)
  elseif(NOT raspimouse_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(raspimouse_FOUND FALSE)
  endif()
  return()
endif()
set(_raspimouse_CONFIG_INCLUDED TRUE)

# output package information
if(NOT raspimouse_FIND_QUIETLY)
  message(STATUS "Found raspimouse: 1.1.1 (${raspimouse_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'raspimouse' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${raspimouse_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(raspimouse_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${raspimouse_DIR}/${_extra}")
endforeach()
