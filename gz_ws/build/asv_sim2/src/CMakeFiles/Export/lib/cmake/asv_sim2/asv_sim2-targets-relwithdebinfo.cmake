#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "asv_sim2::asv_sim2" for configuration "RelWithDebInfo"
set_property(TARGET asv_sim2::asv_sim2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(asv_sim2::asv_sim2 PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libasv_sim2.so.2.0.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libasv_sim2.so.2"
  )

list(APPEND _IMPORT_CHECK_TARGETS asv_sim2::asv_sim2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_asv_sim2::asv_sim2 "${_IMPORT_PREFIX}/lib/libasv_sim2.so.2.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
