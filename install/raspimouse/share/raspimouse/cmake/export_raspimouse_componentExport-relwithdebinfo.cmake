#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "raspimouse::raspimouse_component" for configuration "RelWithDebInfo"
set_property(TARGET raspimouse::raspimouse_component APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(raspimouse::raspimouse_component PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libraspimouse_component.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libraspimouse_component.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS raspimouse::raspimouse_component )
list(APPEND _IMPORT_CHECK_FILES_FOR_raspimouse::raspimouse_component "${_IMPORT_PREFIX}/lib/libraspimouse_component.so" )

# Import target "raspimouse::keyboard_controller" for configuration "RelWithDebInfo"
set_property(TARGET raspimouse::keyboard_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(raspimouse::keyboard_controller PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/bin/keyboard_controller"
  )

list(APPEND _IMPORT_CHECK_TARGETS raspimouse::keyboard_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_raspimouse::keyboard_controller "${_IMPORT_PREFIX}/bin/keyboard_controller" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
