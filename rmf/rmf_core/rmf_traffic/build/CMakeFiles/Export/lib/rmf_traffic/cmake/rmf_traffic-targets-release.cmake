#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rmf_traffic::rmf_traffic" for configuration "Release"
set_property(TARGET rmf_traffic::rmf_traffic APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rmf_traffic::rmf_traffic PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librmf_traffic.so"
  IMPORTED_SONAME_RELEASE "librmf_traffic.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rmf_traffic::rmf_traffic )
list(APPEND _IMPORT_CHECK_FILES_FOR_rmf_traffic::rmf_traffic "${_IMPORT_PREFIX}/lib/librmf_traffic.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
