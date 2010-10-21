# Generate the AIGSConfig.cmake file in the build tree. This doesnot
# configure one for installation. The file tells external projects how to use
# AIGS.

# Help store a literal dollar in a string.  CMake 2.2 allows escaped
# dollars but we have to support CMake 2.0.
SET(DOLLAR "$")

#-----------------------------------------------------------------------------
# Settings for the build tree.

EXPORT_LIBRARY_DEPENDENCIES(
  ${AIGS_BINARY_DIR}/AIGSLibraryDepends.cmake)

# Set the source dir
SET(AIGS_SOURCE_DIR_CONFIG ${AIGS_SOURCE_DIR})

# The library dependencies file.
SET(AIGS_LIBRARY_DEPENDS_FILE
  ${AIGS_BINARY_DIR}/AIGSLibraryDepends.cmake)

INCLUDE(${CMAKE_ROOT}/Modules/CMakeExportBuildSettings.cmake)

CMAKE_EXPORT_BUILD_SETTINGS(
  ${AIGS_BINARY_DIR}/AIGSBuildSettings.cmake)

# The "use" file.
SET(AIGS_USE_FILE_CONFIG
  ${AIGS_BINARY_DIR}/UseAIGS.cmake)

# The build settings file.
SET(AIGS_BUILD_SETTINGS_FILE_CONFIG
  ${AIGS_BINARY_DIR}/AIGSBuildSettings.cmake)

# The library directories.
SET(AIGS_LIBRARY_DIRS_CONFIG ${AIGS_LIBRARY_DIR})

# The kits.
SET(AIGS_KITS_CONFIG ${AIGS_KITS})

# The libraries.
SET(AIGS_LIBRARIES_CONFIG ${AIGS_LIBRARIES})

# The include directories.
SET(AIGS_INCLUDE_DIRS_CONFIG "")
FOREACH(dir ${AIGS_INCLUDE_DIRS})
  SET(AIGS_INCLUDE_DIRS_CONFIG "${AIGS_INCLUDE_DIRS_CONFIG};${dir}")
ENDFOREACH(dir ${AIGS_INCLUDE_DIRS})

# The VTK options.
SET(AIGS_VTK_DIR_CONFIG ${VTK_DIR})

# The library dependencies file.
#IF(NOT AIGS_NO_LIBRARY_DEPENDS)
#  INCLUDE("@AIGS_LIBRARY_DEPENDS_FILE@")
#ENDIF(NOT AIGS_NO_LIBRARY_DEPENDS)

# Configure AIGSConfig.cmake for the build tree.
CONFIGURE_FILE(
  ${AIGS_SOURCE_DIR}/AIGSConfig.cmake.in
  ${AIGS_BINARY_DIR}/AIGSConfig.cmake @ONLY)

# Configure the UseAIGS file
CONFIGURE_FILE(${AIGS_SOURCE_DIR}/UseAIGS.cmake
               ${AIGS_BINARY_DIR}/UseAIGS.cmake COPYONLY)

