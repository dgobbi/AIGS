PROJECT(ATAMAIULTRASOUND)

SET(KIT AtamaiUltrasound)
SET(UKIT ATAMAIULTRASOUND)
SET(KIT_TCL_LIBS vtkHybridTCL )
IF(APPLE OR "${CMAKE_BACKWARDS_COMPATIBILITY}" GREATER "2.3")
  SET(KIT_PYTHON_LIBS vtkAtamaiTrackingPythonD)
ELSE(APPLE OR "${CMAKE_BACKWARDS_COMPATIBILITY}" GREATER "2.3")
  SET(KIT_PYTHON_LIBS vtkAtamaiTrackingPython)
ENDIF(APPLE OR "${CMAKE_BACKWARDS_COMPATIBILITY}" GREATER "2.3")
SET(KIT_JAVA_LIBS vtkAtamaiTrackingJava)
SET(KIT_LIBS vtkAtamaiTracking)

SET ( Kit_SRCS
vtkFreehandUltrasound.cxx
vtkUltrasoundFrameAnalyze.cxx
vtkUltrasoundCompare.cxx
vtkUltrasoundImageStencilSource.cxx
)

INCLUDE_DIRECTORIES(${ATAMAITRACKING_SOURCE_DIR}
                    ${ATAMAITRACKING_BINARY_DIR})

SET(Kit_EXTRA_SRCS)
SET(Kit_EXTRA_CMDS)
SET(Kit_TCL_EXTRA_SRCS)
SET(Kit_PYTHON_EXTRA_SRCS)
SET(Kit_JAVA_EXTRA_SRCS)

# -----------------------------------------------------------------

# Include the common setting file
INCLUDE(${AtamaiVTK_SOURCE_DIR}/CMake/KitCommonBlock.cmake)
