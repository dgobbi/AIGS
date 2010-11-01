#
# This module is provided as AIGS_USE_FILE by AIGSConfig.cmake.
# It can be INCLUDEd in a project to load the needed compiler and linker
# settings to use AIGS:
#   FIND_PACKAGE(AIGS REQUIRED)
#   INCLUDE(${AIGS_USE_FILE})
#

IF(NOT AIGS_USE_FILE_INCLUDED)
  SET(AIGS_USE_FILE_INCLUDED 1)

  # Add include directories needed to use AIGS.
  INCLUDE_DIRECTORIES(${AIGS_INCLUDE_DIRS})

  # Add link directories needed to use AIGS.
  LINK_DIRECTORIES(${AIGS_LIBRARY_DIRS})

  #
  # VTK
  #
  IF(NOT VTK_DIR)
    # Use AIGS_VTK_DIR or find a new one
    IF(AIGS_VTK_DIR)
      SET(VTK_DIR ${AIGS_VTK_DIR} CACHE PATH "Path to VTK build dir")
      INCLUDE(${VTK_DIR}/VTKConfig.cmake)
    ELSE(AIGS_VTK_DIR)
      FIND_PACKAGE(VTK REQUIRED)
    ENDIF(AIGS_VTK_DIR)
  ELSE(NOT VTK_DIR)
    INCLUDE(${VTK_DIR}/VTKConfig.cmake)
  ENDIF(NOT VTK_DIR)

  # Include the VTK use file
  INCLUDE(${VTK_USE_FILE})

ENDIF(NOT AIGS_USE_FILE_INCLUDED)
