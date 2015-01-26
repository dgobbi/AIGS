#PROJECT(Ultrasound)

SET(KIT Ultrasound)
SET(UKIT Ultrasound)
IF(${VTK_MAJOR_VERSION} VERSION_LESS 6)
  SET(KIT_LIBS vtkHybrid vtkTracking)
ELSE(${VTK_MAJOR_VERSION} VERSION_LESS 6)
  SET(KIT_LIBS
      vtkCommonCore vtkCommonSystem vtkCommonMath vtkCommonTransforms
      vtkCommonDataModel vtkCommonExecutionModel)
ENDIF(${VTK_MAJOR_VERSION} VERSION_LESS 6)

SET(VTK_WRAP_HINTS ${CMAKE_CURRENT_SOURCE_DIR}/hints)

SET ( Kit_SRCS
vtkFreehandUltrasound.cxx
#vtkFreehandUltrasound2.cxx
vtkUltrasoundCompare.cxx
vtkUltrasoundFan.cxx
vtkUltrasoundFrameAnalyze.cxx
vtkUltrasoundImageStencilSource.cxx
)

SET(Kit_EXTRA_SRCS)
SET(Kit_EXTRA_CMDS)
SET(Kit_TCL_EXTRA_SRCS)
SET(Kit_PYTHON_EXTRA_SRCS)
SET(Kit_JAVA_EXTRA_SRCS)

# -----------------------------------------------------------------
# Include the common setting file
INCLUDE(${AIGS_SOURCE_DIR}/CMake/AIGSCommonBlock.cmake)
