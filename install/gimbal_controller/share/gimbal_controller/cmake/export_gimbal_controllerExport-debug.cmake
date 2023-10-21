#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "gimbal_controller::gimbal_controller" for configuration "Debug"
set_property(TARGET gimbal_controller::gimbal_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(gimbal_controller::gimbal_controller PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libgimbal_controller.so"
  IMPORTED_SONAME_DEBUG "libgimbal_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS gimbal_controller::gimbal_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_gimbal_controller::gimbal_controller "${_IMPORT_PREFIX}/lib/libgimbal_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
