PROJECT(ATAMAITRACKING)

SET(KIT AtamaiTracking)
SET(UKIT ATAMAITRACKING)
SET(KIT_LIBS vtkHybrid)
SET(KIT_EXTRA_LIBS)

SET ( Kit_SRCS
vtkTracker.cxx
vtkTrackerTool.cxx
vtkTrackerBuffer.cxx
vtkFrameToTimeConverter.cxx
)

SET_SOURCE_FILES_PROPERTIES(
vtkTracker
ABSTRACT
)

IF (VTK_USE_POLARIS)
  SET ( Kit_SRCS ${Kit_SRCS}
        vtkPOLARISTracker.cxx
	vtkNDITracker.cxx)
  SET (KIT_EXTRA_LIBS ${KIT_EXTRA_LIBS} vtkndicapi)
  INCLUDE_DIRECTORIES(${VTKNDICAPI_SOURCE_DIR}
                      ${VTKNDICAPI_BINARY_DIR})
ENDIF (VTK_USE_POLARIS)

IF (VTK_USE_FLOCK)
  SET ( Kit_SRCS ${Kit_SRCS}
        vtkFlockTracker.cxx)
  SET (KIT_EXTRA_LIBS ${KIT_EXTRA_LIBS} vtkflock)
  INCLUDE_DIRECTORIES(${VTKFLOCK_SOURCE_DIR}
                      ${VTKFLOCK_BINARY_DIR})
ENDIF (VTK_USE_FLOCK)

IF (VTK_USE_LOGITECH)
  SET ( Kit_SRCS ${Kit_SRCS}
        vtkLogitechTracker.cxx)
  SET (KIT_EXTRA_LIBS ${KIT_EXTRA_LIBS} vtklogitech3d)
  INCLUDE_DIRECTORIES(${VTKLOGITECH_SOURCE_DIR}
                      ${VTKLOGITECH_BINARY_DIR})
ENDIF (VTK_USE_LOGITECH)

IF (VTK_USE_POLHEMUS)
  SET ( Kit_SRCS ${Kit_SRCS}
        vtkPolhemusTracker.cxx)
  SET (KIT_EXTRA_LIBS ${KIT_EXTRA_LIBS} vtkpolhemus)
  INCLUDE_DIRECTORIES(${VTKPOLHEMUS_SOURCE_DIR}
                      ${VTKPOLHEMUS_BINARY_DIR})
ENDIF (VTK_USE_POLHEMUS)

IF (VTK_USE_MICRONTRACKER)
  SET ( AtamaiTracking_SRCS ${AtamaiTracking_SRCS}
	vtkMicronTracker.cxx)
  INCLUDE_DIRECTORIES(${VTKMICRONTRACKERFILES_SOURCE_DIR}
                      ${VTKMICRONTRACKERFILES_BINARY_DIR})
  TARGET_LINK_LIBRARIES (vtkAtamaiTracking vtkMicrontrackerfiles)
  INCLUDE (${CMAKE_ROOT}/Modules/FindOpenGL.cmake)
ENDIF (VTK_USE_MICRONTRACKER)

SET(Kit_EXTRA_SRCS)
SET(Kit_EXTRA_CMDS)
SET(Kit_TCL_EXTRA_SRCS)
SET(Kit_PYTHON_EXTRA_SRCS)
SET(Kit_JAVA_EXTRA_SRCS)

# -----------------------------------------------------------------

# Include the common setting file
INCLUDE(${AtamaiVTK_SOURCE_DIR}/CMake/KitCommonBlock.cmake)