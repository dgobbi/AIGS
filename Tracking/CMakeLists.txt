PROJECT(ATAMAITRACKING)

SET(KIT AtamaiTracking)
SET(UKIT ATAMAITRACKING)
SET(KIT_TCL_LIBS vtkHybridTCL )
IF(APPLE OR "${CMAKE_BACKWARDS_COMPATIBILITY}" GREATER "2.3")
  SET(KIT_PYTHON_LIBS vtkHybridPythonD)
ELSE(APPLE OR "${CMAKE_BACKWARDS_COMPATIBILITY}" GREATER "2.3")
  SET(KIT_PYTHON_LIBS vtkHybridPython)
ENDIF(APPLE OR "${CMAKE_BACKWARDS_COMPATIBILITY}" GREATER "2.3")
SET(KIT_JAVA_LIBS vtkHybridJava)
SET(KIT_LIBS vtkHybrid)

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
  SET (KIT_LIBS ${KIT_LIBS} vtkndicapi)
  INCLUDE_DIRECTORIES(${VTKNDICAPI_SOURCE_DIR}
                      ${VTKNDICAPI_BINARY_DIR})
ENDIF (VTK_USE_POLARIS)

IF (VTK_USE_FLOCK)
  SET ( Kit_SRCS ${Kit_SRCS}
        vtkFlockTracker.cxx)
  SET (KIT_LIBS ${KIT_LIBS} vtkflock)
  INCLUDE_DIRECTORIES(${VTKFLOCK_SOURCE_DIR}
                      ${VTKFLOCK_BINARY_DIR})
ENDIF (VTK_USE_FLOCK)

IF (VTK_USE_LOGITECH)
  SET ( Kit_SRCS ${Kit_SRCS}
        vtkLogitechTracker.cxx)
  SET (KIT_LIBS ${KIT_LIBS} vtklogitech3d)
  INCLUDE_DIRECTORIES(${VTKLOGITECH_SOURCE_DIR}
                      ${VTKLOGITECH_BINARY_DIR})
ENDIF (VTK_USE_LOGITECH)

IF (VTK_USE_POLHEMUS)
  SET ( Kit_SRCS ${Kit_SRCS}
        vtkPolhemusTracker.cxx)
  SET (KIT_LIBS ${KIT_LIBS} vtkpolhemus)
  INCLUDE_DIRECTORIES(${VTKPOLHEMUS_SOURCE_DIR}
                      ${VTKPOLHEMUS_BINARY_DIR})
ENDIF (VTK_USE_POLHEMUS)

SET(Kit_EXTRA_SRCS)
SET(Kit_EXTRA_CMDS)
SET(Kit_TCL_EXTRA_SRCS)
SET(Kit_PYTHON_EXTRA_SRCS)
SET(Kit_JAVA_EXTRA_SRCS)

# -----------------------------------------------------------------
# Start of common section

# Need to include class headers and the configuration header.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})

## Setup vtkInstantiator registration for this library's classes.
#VTK_MAKE_INSTANTIATOR2(vtk${KIT}Instantiator KitInstantiator_SRCS
#                       ${Kit_SRCS}
#                       EXPORT_MACRO VTK_${UKIT}_EXPORT
#                       HEADER_LOCATION ${ATAMAIVTK_BINARY_DIR})

#ADD_LIBRARY(vtk${KIT} ${Kit_SRCS} ${Kit_EXTRA_SRCS} ${KitInstantiator_SRCS})

ADD_LIBRARY(vtk${KIT} ${Kit_SRCS} ${Kit_EXTRA_SRCS})
#IF(VTK_APPLE_SHARED_FLAGS_NEEDED)
#  SET_TARGET_PROPERTIES(vtk${KIT} PROPERTIES LINK_FLAGS "-init _oft_init${KIT}")
#ENDIF(VTK_APPLE_SHARED_FLAGS_NEEDED)

# Allow the user to customize their build with some local options
#

INCLUDE (${ATAMAIVTK_BINARY_DIR}/${KIT}/LocalUserOptions.cmake OPTIONAL)
INCLUDE (${ATAMAIVTK_SOURCE_DIR}/${KIT}/LocalUserOptions.cmake OPTIONAL)

# if we are wrapping into Tcl then add the library and extra
# source files
#
IF (VTK_WRAP_TCL)
  VTK_WRAP_TCL2(vtk${KIT}TCL SOURCES KitTCL_SRCS 
                ${Kit_SRCS}
                COMMANDS ${Kit_TCL_EXTRA_CMDS})
  ADD_LIBRARY(vtk${KIT}TCL ${KitTCL_SRCS} ${Kit_TCL_EXTRA_SRCS})
  TARGET_LINK_LIBRARIES (vtk${KIT}TCL vtk${KIT} ${KIT_TCL_LIBS})
  INSTALL_TARGETS(/lib/vtk vtk${KIT}TCL)
ENDIF (VTK_WRAP_TCL)

# if we are wrapping into Python then add the library and extra
# source files
#
IF (VTK_WRAP_PYTHON)
  VTK_WRAP_PYTHON2(vtk${KIT}Python KitPython_SRCS ${Kit_SRCS})
  IF (APPLE)
    ADD_LIBRARY(vtk${KIT}PythonD ${KitPython_SRCS} ${Kit_PYTHON_EXTRA_SRCS})
    ADD_LIBRARY(vtk${KIT}Python MODULE vtk${KIT}PythonInit.cxx)
    TARGET_LINK_LIBRARIES(vtk${KIT}PythonD vtk${KIT} ${KIT_PYTHON_LIBS} ${PYTHON_LIBRARY})
    TARGET_LINK_LIBRARIES(vtk${KIT}Python vtk${KIT}PythonD)
    INSTALL_TARGETS(/lib/vtk vtk${KIT}PythonD)
  ELSE (APPLE)
    ADD_LIBRARY(vtk${KIT}Python MODULE ${KitPython_SRCS} ${Kit_PYTHON_EXTRA_SRCS})
    TARGET_LINK_LIBRARIES (vtk${KIT}Python vtk${KIT} ${KIT_PYTHON_LIBS})
  ENDIF(APPLE)
  IF(WIN32)
    TARGET_LINK_LIBRARIES (vtk${KIT}Python
                           debug ${PYTHON_DEBUG_LIBRARY}
                           optimized ${PYTHON_LIBRARY})
  ENDIF(WIN32)
  INSTALL_TARGETS(/lib/vtk vtk${KIT}Python)
ENDIF (VTK_WRAP_PYTHON)

# if we are wrapping into Java then add the library and extra
# source files
#
IF (VTK_WRAP_JAVA)
  VTK_WRAP_JAVA2(vtk${KIT}Java KitJava_SRCS ${Kit_SRCS})
  ADD_LIBRARY(vtk${KIT}Java SHARED ${KitJava_SRCS} ${Kit_JAVA_EXTRA_SRCS})
  TARGET_LINK_LIBRARIES(vtk${KIT}Java vtk${KIT} ${KIT_JAVA_LIBS})
  INSTALL_TARGETS(/lib/vtk vtk${KIT}Java)
ENDIF (VTK_WRAP_JAVA)

TARGET_LINK_LIBRARIES(vtk${KIT} ${KIT_LIBS})

INSTALL_TARGETS(/lib/vtk vtk${KIT})
INSTALL_FILES(/include/vtk .h ${Kit_SRCS})

# If the user defined a custom macro, execute it now and pass in all the srcs
#
IF(LOCALUSERMACRODEFINED)
  LocalUserOptionsMacro( "${Kit_SRCS}"       "${Kit_EXTRA_SRCS}"
                         "${KitTCL_SRCS}"    "${Kit_TCL_EXTRA_SRCS}"
                         "${KitJava_SRCS}"   "${Kit_JAVA_EXTRA_SRCS}"
                         "${KitPython_SRCS}" "${Kit_PYTHON_EXTRA_SRCS}")
ENDIF(LOCALUSERMACRODEFINED)

# End of common section
# -----------------------------------------------------------------
