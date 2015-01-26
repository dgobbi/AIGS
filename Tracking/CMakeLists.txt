#PROJECT(Tracking)

SET(KIT Tracking)
SET(UKIT Tracking)
IF(${VTK_MAJOR_VERSION} VERSION_LESS 6)
  SET(KIT_LIBS vtkHybrid vtkParallel)
ELSE(${VTK_MAJOR_VERSION} VERSION_LESS 6)
  SET(KIT_LIBS
      vtkCommonCore vtkCommonSystem vtkCommonMath vtkCommonTransforms
      vtkParallelCore)
ENDIF(${VTK_MAJOR_VERSION} VERSION_LESS 6)

SET(VTK_WRAP_HINTS ${CMAKE_CURRENT_SOURCE_DIR}/hints)

SET ( Kit_SRCS
vtkTracker.cxx
vtkTrackerTool.cxx
vtkFakeTracker.cxx
vtkTrackerBuffer.cxx
vtkFrameToTimeConverter.cxx
)

SET_SOURCE_FILES_PROPERTIES(
vtkTracker
ABSTRACT
)

IF (AIGS_USE_CERTUS)
  SET ( Kit_SRCS ${Kit_SRCS}
	vtkNDICertusTracker.cxx)
  FIND_PATH(NDIOAPI_INCLUDE_DIR ndopto.h)
  FIND_LIBRARY(NDIOAPI_LIBRARY oapi)
  INCLUDE_DIRECTORIES(${VTKNDICAPI_SOURCE_DIR}
                      ${VTKNDICAPI_BINARY_DIR}
                      ${NDIOAPI_INCLUDE_DIR})
  SET (KIT_EXTRA_LIBS ${KIT_EXTRA_LIBS} vtkndicapi ${NDIOAPI_LIBRARY})
ENDIF (AIGS_USE_CERTUS)

IF (AIGS_USE_NDI)
  SET ( Kit_SRCS ${Kit_SRCS}
	vtkNDITracker.cxx)
  SET (KIT_EXTRA_LIBS ${KIT_EXTRA_LIBS} vtkndicapi)
  INCLUDE_DIRECTORIES(${VTKNDICAPI_SOURCE_DIR}
                      ${VTKNDICAPI_BINARY_DIR})
ENDIF (AIGS_USE_NDI)

IF (AIGS_USE_FLOCK)
  SET ( Kit_SRCS ${Kit_SRCS}
        vtkFlockTracker.cxx)
  SET (KIT_EXTRA_LIBS ${KIT_EXTRA_LIBS} vtkflock)
  INCLUDE_DIRECTORIES(${VTKFLOCK_SOURCE_DIR}
                      ${VTKFLOCK_BINARY_DIR})
ENDIF (AIGS_USE_FLOCK)

IF (AIGS_USE_LOGITECH)
  SET ( Kit_SRCS ${Kit_SRCS}
        vtkLogitechTracker.cxx)
  SET (KIT_EXTRA_LIBS ${KIT_EXTRA_LIBS} vtklogitech3d)
  INCLUDE_DIRECTORIES(${VTKLOGITECH_SOURCE_DIR}
                      ${VTKLOGITECH_BINARY_DIR})
ENDIF (AIGS_USE_LOGITECH)

IF (AIGS_USE_POLHEMUS)
  SET ( Kit_SRCS ${Kit_SRCS}
        vtkPolhemusTracker.cxx)
  SET (KIT_EXTRA_LIBS ${KIT_EXTRA_LIBS} vtkpolhemus)
  INCLUDE_DIRECTORIES(${VTKPOLHEMUS_SOURCE_DIR}
                      ${VTKPOLHEMUS_BINARY_DIR})
ENDIF (AIGS_USE_POLHEMUS)

IF (AIGS_USE_MICRONTRACKER)
  SET ( Tracking_SRCS ${Tracking_SRCS}
	vtkMicronTracker.cxx)
  INCLUDE_DIRECTORIES(${VTKMICRONTRACKERFILES_SOURCE_DIR}
                      ${VTKMICRONTRACKERFILES_BINARY_DIR})
  TARGET_LINK_LIBRARIES (vtkTracking vtkMicrontrackerfiles)
  INCLUDE (${CMAKE_ROOT}/Modules/FindOpenGL.cmake)
ENDIF (AIGS_USE_MICRONTRACKER)

SET(Kit_EXTRA_SRCS)
SET(Kit_EXTRA_CMDS)
SET(Kit_TCL_EXTRA_SRCS)
SET(Kit_PYTHON_EXTRA_SRCS)
SET(Kit_JAVA_EXTRA_SRCS)

# -----------------------------------------------------------------
# Include the common setting file
INCLUDE(${AIGS_SOURCE_DIR}/CMake/AIGSCommonBlock.cmake)
