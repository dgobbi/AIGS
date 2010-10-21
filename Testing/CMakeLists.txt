PROJECT(AIGSTesting)

# Testing out of AIGS build tree
IF (NOT AIGS_BINARY_DIR)
  MESSAGE(STATUS "Buiding AIGSTesting out of AIGS build tree")

  IF (NOT AIGS_DIR)
    FIND_PACKAGE(AIGS REQUIRED)
  ELSE(NOT AIGS_DIR)
    INCLUDE(${AIGS_DIR}/AIGSConfig.cmake)
  ENDIF (NOT AIGS_DIR)

  INCLUDE(${AIGS_USE_FILE})

  # Test output
  SET(AIGSTesting_TEMP_DIR ${AIGSTesting_BINARY_DIR}/Testing/Temporary)

  # Configure Dart testing support.
  INCLUDE(${CMAKE_ROOT}/Modules/Dart.cmake)

  # Output paths
  SET(EXECUTABLE_OUTPUT_PATH ${AIGSTesting_BINARY_DIR}/bin/ CACHE INTERNAL "Single output directory for building all executables.")
  SET(AIGSTesting_EXECUTABLE_DIR ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_CFG_INTDIR})
  SET(CXX_TEST_PATH ${AIGSTesting_EXECUTABLE_DIR})

# Testing inside the AIGS build tree
ELSE (NOT AIGS_BINARY_DIR)
  MESSAGE(STATUS "Buiding AIGSTesting inside the AIGS build tree")

  INCLUDE_DIRECTORIES(${AIGS_INCLUDE_DIRS})

  # Test output
  SET(AIGSTesting_TEMP_DIR ${AIGS_BINARY_DIR}/Testing/Temporary)
ENDIF (NOT AIGS_BINARY_DIR)

#
# Enable testing
#
#ENABLE_TESTING()
