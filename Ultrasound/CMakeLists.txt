#PROJECT(ATAMAIULTRASOUND)

SET(KIT AtamaiUltrasound)
SET(UKIT ATAMAIULTRASOUND)
SET(KIT_LIBS vtkHybrid vtkAtamaiTracking)

IF(VTK_MAJOR_VERSION LESS 5)
  SET(Kit_SRCS ${Kit_SRCS}
    vtkFreehandUltrasound.cxx
    vtkUltrasoundCompare.cxx
    vtkUltrasoundFrameAnalyze.cxx
    vtkUltrasoundImageStencilSource.cxx
  )
ELSE(VTK_MAJOR_VERSION LESS 5)
  SET(Kit_SRCS ${Kit_SRCS}
  )
ENDIF(VTK_MAJOR_VERSION LESS 5)

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
