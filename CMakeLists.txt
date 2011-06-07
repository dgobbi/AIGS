PROJECT(AIGS)

# CMake version requirements
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

# For CMake 2.6
IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND CMAKE_POLICY)

# Look for a VTK build directory
FIND_PACKAGE(VTK)
IF (VTK_FOUND)
  INCLUDE (${VTK_USE_FILE})
ENDIF (VTK_FOUND)

# Choose static or shared libraries.
OPTION(BUILD_SHARED_LIBS "Build shared libraries." OFF)
SET (AIGS_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
IF (BUILD_SHARED_LIBS)
  SET (BUILD_SHARED_LIBS_CONFIG 1)
ELSE (BUILD_SHARED_LIBS)
  SET (BUILD_SHARED_LIBS_CONFIG 0)
ENDIF (BUILD_SHARED_LIBS)

# Build documentation
OPTION(BUILD_DOCUMENTATION "Build Documentation" OFF)
IF(BUILD_DOCUMENTATION)
  FIND_PACKAGE(VTK)
  IF (USE_VTK_FILE)
    INCLUDE (${USE_VTK_FILE})
  ELSE (USE_VTK_FILE)
    MESSAGE(FATAL_ERROR "This application requires VTK.")
  ENDIF (USE_VTK_FILE)
  ADD_SUBDIRECTORY(Documentation/Doxygen)
ENDIF(BUILD_DOCUMENTATION)

# Output directories.
SET(LIBRARY_OUTPUT_PATH ${AIGS_BINARY_DIR}/bin CACHE INTERNAL "Single output directory for building all libraries.")
SET(EXECUTABLE_OUTPUT_PATH ${AIGS_BINARY_DIR}/bin CACHE INTERNAL "Single output directory for building all executables.")
SET(AIGS_LIBRARY_DIR ${LIBRARY_OUTPUT_PATH}/${CMAKE_CFG_INTDIR})
SET(AIGS_EXECUTABLE_DIR ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_CFG_INTDIR})
SET(CXX_TEST_PATH ${EXECUTABLE_OUTPUT_PATH})

# Configure Dart testing support.
INCLUDE(${CMAKE_ROOT}/Modules/Dart.cmake)

# ---------------------------------------------------------------
# Initialize configuration

# Kits
SET(AIGS_KITS)

# Libraries
SET(AIGS_LIBRARIES)

# Include directories
SET(AIGS_INCLUDE_DIRS)

# ---------------------------------------------------------------
# Include the Tcl and Python wrappers
IF(VTK_WRAP_TCL AND ${VTK_MAJOR_VERSION} GREATER 4)
  INCLUDE(${VTK_CMAKE_DIR}/vtkWrapTcl.cmake)
ENDIF(VTK_WRAP_TCL AND ${VTK_MAJOR_VERSION} GREATER 4)

IF(VTK_WRAP_PYTHON AND ${VTK_MAJOR_VERSION} GREATER 4)
  IF(VTK_PYTHON_INCLUDE_DIR)
    INCLUDE_DIRECTORIES("${VTK_PYTHON_INCLUDE_DIR}")
  ENDIF(VTK_PYTHON_INCLUDE_DIR)
  INCLUDE(${VTK_CMAKE_DIR}/vtkWrapPython.cmake)
ENDIF(VTK_WRAP_PYTHON AND ${VTK_MAJOR_VERSION} GREATER 4)

IF(VTK_WRAP_JAVA AND ${VTK_MAJOR_VERSION} GREATER 4)
  SET(VTK_WRAP_JAVA 0)
ENDIF(VTK_WRAP_JAVA AND ${VTK_MAJOR_VERSION} GREATER 4)

# Include AIGSThirdParty
INCLUDE(${AIGS_SOURCE_DIR}/CMake/AIGSThirdParty.cmake)

# ---------------------------------------------------------------
# Start listing the AIGS kits

# Tracking
OPTION (AIGS_USE_TRACKING "Build the Tracking module" ON)
IF (AIGS_USE_TRACKING)
  OPTION (AIGS_USE_NDI "Provide support for the NDI POLARIS and AURORA" ON)
  OPTION (AIGS_USE_CERTUS "Provide support for the NDI Certus" OFF)
  OPTION (AIGS_USE_FLOCK "Provide support for the Ascension Flock of Birds" ON)
  OPTION (AIGS_USE_LOGITECH "Provide support for the Logitech 3D mouse" OFF)
  OPTION (AIGS_USE_POLHEMUS "Provide support for the Polhemus FasTrak" OFF)
  OPTION (AIGS_USE_MICRONTRACKER "Provide support for the Claron MicronTracker" OFF)

  SET(AIGS_INCLUDE_DIRS ${AIGS_INCLUDE_DIRS}
      ${AIGS_SOURCE_DIR}/Tracking)
  ADD_SUBDIRECTORY(Utilities)
  ADD_SUBDIRECTORY(Tracking)

  SET(AIGS_LIBRARIES ${AIGS_LIBRARIES} Tracking)
ENDIF (AIGS_USE_TRACKING)

# Ultrasound
IF (AIGS_USE_ULTRASOUND)
  SET(AIGS_INCLUDE_DIRS ${AIGS_INCLUDE_DIRS}
      ${AIGS_SOURCE_DIR}/Ultrasound)
  ADD_SUBDIRECTORY(Ultrasound)

  SET(AIGS_LIBRARIES ${AIGS_LIBRARIES} Ultrasound)
ENDIF (AIGS_USE_ULTRASOUND)

# Build Testing
OPTION(BUILD_TESTING "Build the tests" OFF)
IF(BUILD_TESTING)
   ADD_SUBDIRECTORY(Testing)
ENDIF(BUILD_TESTING)

# Build Examples
OPTION(BUILD_EXAMPLES "Build the examples" OFF)
IF (BUILD_EXAMPLES)
   ADD_SUBDIRECTORY(Examples)
ENDIF (BUILD_EXAMPLES)

# Add aigs.py file.  On Win32, this has to go into all config dirs
IF(VTK_WRAP_PYTHON)
  IF(WIN32 AND CMAKE_CONFIGURATION_TYPES)
    FOREACH(config ${CMAKE_CONFIGURATION_TYPES})
      CONFIGURE_FILE(
        ${AIGS_SOURCE_DIR}/AIGS.py.in
        ${LIBRARY_OUTPUT_PATH}/${config}/aigs.py)
    ENDFOREACH(config)
  ELSE(WIN32 AND CMAKE_CONFIGURATION_TYPES)
    CONFIGURE_FILE(
      ${AIGS_SOURCE_DIR}/aigs.py.in
      ${LIBRARY_OUTPUT_PATH}/aigs.py)
  ENDIF(WIN32 AND CMAKE_CONFIGURATION_TYPES)
ENDIF(VTK_WRAP_PYTHON)

#-----------------------------------------------------------------------------
# The commands in this directory are intended to be executed as
# the end of the whole configuration process, as a "last step".
# This directory is typically the last SUBDIRS in the main CMakeLists.txt.
ADD_SUBDIRECTORY(CMake/LastConfigureStep)

