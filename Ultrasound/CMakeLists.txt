#PROJECT(ULTRASOUND)

SET(KIT Ultrasound)
SET(UKIT ULTRASOUND)
SET(KIT_LIBS vtkHybrid vtkAtamaiTracking)

IF(VTK_MAJOR_VERSION LESS 5)
  SET(Kit_SRCS ${Kit_SRCS}
  )
ELSE(VTK_MAJOR_VERSION LESS 5)
  SET(Kit_SRCS ${Kit_SRCS}
    vtkFreehandUltrasound.cxx
    #vtkFreehandUltrasound2.cxx
    vtkUltrasoundCompare.cxx
    vtkUltrasoundFrameAnalyze.cxx
    vtkUltrasoundImageStencilSource.cxx
  )
ENDIF(VTK_MAJOR_VERSION LESS 5)

INCLUDE_DIRECTORIES(${TRACKING_SOURCE_DIR}
                    ${TRACKING_BINARY_DIR})

SET(Kit_EXTRA_SRCS)
SET(Kit_EXTRA_CMDS)
SET(Kit_TCL_EXTRA_SRCS)
SET(Kit_PYTHON_EXTRA_SRCS)
SET(Kit_JAVA_EXTRA_SRCS)

# -----------------------------------------------------------------

# Include the common setting file
INCLUDE(${AIGS_SOURCE_DIR}/CMake/AIGSCommonBlock.cmake)
