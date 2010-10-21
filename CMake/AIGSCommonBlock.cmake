# Need to include class headers and the configuration header.
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

# Add this kit to the configuration variables
IF(AtamaiVTK_KITS)
  STRING(REGEX MATCH "${KIT}" already_have_kit ${AtamaiVTK_KITS})
ELSE(AtamaiVTK_KITS)
  SET(already_have_kit 0)
ENDIF(AtamaiVTK_KITS)

IF(NOT already_have_kit)
  SET(AtamaiVTK_KITS ${AtamaiVTK_KITS} ${KIT}
    CACHE INTERNAL "AtamaiVTK Kits" FORCE)

  SET(AtamaiVTK_LIBRARIES ${AtamaiVTK_LIBRARIES} vtk${KIT}
    CACHE INTERNAL "AtamaiVTK libraries" FORCE)

  SET(AtamaiVTK_INCLUDE_DIRS ${AtamaiVTK_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
    CACHE INTERNAL "AtamaiVTK libraries" FORCE)
ENDIF(NOT already_have_kit)

# Add the wrapper libraries
SET(KIT_TCL_LIBS "")
SET(KIT_PYTHON_LIBS "")
SET(KIT_JAVA_LIBS "")

# Add the kit libraries to the wrapper libraries
FOREACH( lib ${KIT_LIBS})
  SET(KIT_TCL_LIBS ${KIT_TCL_LIBS} "${lib}TCL")

  IF(APPLE OR ${VTK_MAJOR_VERSION} GREATER 4)
    SET(KIT_PYTHON_LIBS ${KIT_PYTHON_LIBS} "${lib}PythonD")
  ELSE(APPLE OR ${VTK_MAJOR_VERSION} GREATER 4)
    SET(KIT_PYTHON_LIBS ${KIT_PYTHON_LIBS} "${lib}Python")
  ENDIF(APPLE OR ${VTK_MAJOR_VERSION} GREATER 4)

  SET(KIT_JAVA_LIBS ${KIT_JAVA_LIBS} "${lib}Java")
ENDFOREACH( lib ${KIT_LIBS})

## Setup vtkInstantiator registration for this library's classes.
#VTK_MAKE_INSTANTIATOR3(vtk${KIT}Instantiator KitInstantiator_SRCS
#                       ${Kit_SRCS}
#                       VTK_${UKIT}_EXPORT
#                       ${VTK_BINARY_DIR} "")

#ADD_LIBRARY(vtk${KIT} ${Kit_SRCS} ${Kit_EXTRA_SRCS} ${KitInstantiator_SRCS})
ADD_LIBRARY(vtk${KIT} ${Kit_SRCS} ${Kit_EXTRA_SRCS})

# Allow the user to customize their build with some local options
#
SET(LOCALUSERMACRODEFINED 0)
INCLUDE (${VTK_BINARY_DIR}/${KIT}/LocalUserOptions.cmake OPTIONAL)
INCLUDE (${VTK_SOURCE_DIR}/${KIT}/LocalUserOptions.cmake OPTIONAL)

# Do the TCL wrapping
#
IF(VTK_WRAP_TCL)
  IF(VTK_MAJOR_VERSION GREATER 4)
    # First, the VTK 5 style wrapping
    #
    VTK_WRAP_TCL3(vtk${KIT}TCL KitTCL_SRCS
      "${Kit_SRCS}"
      "${Kit_TCL_EXTRA_CMDS}")
    ADD_LIBRARY(vtk${KIT}TCL ${KitTCL_SRCS} ${Kit_TCL_EXTRA_SRCS})
    SET(KIT_LIBRARY_TARGETS ${KIT_LIBRARY_TARGETS} vtk${KIT}TCL)
    TARGET_LINK_LIBRARIES (vtk${KIT}TCL vtk${KIT} ${KIT_TCL_LIBS})
    IF(KIT_TCL_DEPS)
      ADD_DEPENDENCIES(vtk${KIT}TCL ${KIT_TCL_DEPS})
    ENDIF(KIT_TCL_DEPS)
  ELSE(VTK_MAJOR_VERSION GREATER 4)
    # Do the VTK 4 style wrapping
    #
    VTK_WRAP_TCL2(vtk${KIT}TCL SOURCES KitTCL_SRCS
      "${Kit_SRCS}"
      COMMANDS "${Kit_TCL_EXTRA_CMDS}")
    ADD_LIBRARY(vtk${KIT}TCL ${KitTCL_SRCS} ${Kit_TCL_EXTRA_SRCS})
    TARGET_LINK_LIBRARIES (vtk${KIT}TCL vtk${KIT} ${KIT_TCL_LIBS})
    INSTALL_TARGETS(/lib/vtk vtk${KIT}TCL)
    SUBDIR_DEPENDS(Wrapping/Tcl vtk${KIT})
  ENDIF(VTK_MAJOR_VERSION GREATER 4)
ENDIF(VTK_WRAP_TCL)

# Do the Python wrapping
#
IF(VTK_WRAP_PYTHON)
  IF(VTK_MAJOR_VERSION GREATER 4)
    # First, the VTK 5 style wrapping
    #
    VTK_WRAP_PYTHON3(vtk${KIT}Python KitPython_SRCS "${Kit_SRCS}")
    ADD_LIBRARY(vtk${KIT}PythonD ${KitPython_SRCS} ${Kit_PYTHON_EXTRA_SRCS})
    TARGET_LINK_LIBRARIES(vtk${KIT}PythonD vtk${KIT} ${KIT_PYTHON_LIBS})
    SET(KIT_LIBRARY_TARGETS ${KIT_LIBRARY_TARGETS} vtk${KIT}PythonD)
    # On Win32 and Mac, link python library now, on UNIX, don't
    IF(WIN32 OR APPLE)
      TARGET_LINK_LIBRARIES (vtk${KIT}PythonD ${VTK_PYTHON_LIBRARIES})
    ENDIF(WIN32 OR APPLE)
    IF(KIT_PYTHON_DEPS)
      ADD_DEPENDENCIES(vtk${KIT}PythonD ${KIT_PYTHON_DEPS})
    ENDIF(KIT_PYTHON_DEPS)
    ADD_LIBRARY(vtk${KIT}Python MODULE vtk${KIT}PythonInit.cxx)
    SET_TARGET_PROPERTIES(vtk${KIT}Python PROPERTIES PREFIX "")
    IF(WIN32 AND NOT CYGWIN)
      SET_TARGET_PROPERTIES(vtk${KIT}Python PROPERTIES SUFFIX ".pyd")
    ENDIF(WIN32 AND NOT CYGWIN)

    TARGET_LINK_LIBRARIES(vtk${KIT}Python vtk${KIT}PythonD)
    # On Win32 and Mac, link python library now, on UNIX, don't
    IF(WIN32 OR APPLE)
      TARGET_LINK_LIBRARIES (vtk${KIT}Python ${VTK_PYTHON_LIBRARIES})
    ENDIF(WIN32 OR APPLE)
  ELSE(VTK_MAJOR_VERSION GREATER 4)
    # Do the VTK 4 style wrapping
    #
    VTK_WRAP_PYTHON2(vtk${KIT}Python KitPython_SRCS ${Kit_SRCS})
    IF (APPLE)
      ADD_LIBRARY(vtk${KIT}PythonD ${KitPython_SRCS} ${Kit_PYTHON_EXTRA_SRCS})
      ADD_LIBRARY(vtk${KIT}Python MODULE vtk${KIT}PythonInit.cxx)
      TARGET_LINK_LIBRARIES(vtk${KIT}PythonD vtk${KIT} ${KIT_PYTHON_LIBS}
        ${PYTHON_LIBRARY})
      TARGET_LINK_LIBRARIES(vtk${KIT}Python vtk${KIT}PythonD)
      INSTALL_TARGETS(/lib/vtk vtk${KIT}PythonD)
    ELSE (APPLE)
      ADD_LIBRARY(vtk${KIT}Python MODULE ${KitPython_SRCS}
        ${Kit_PYTHON_EXTRA_SRCS})
      TARGET_LINK_LIBRARIES (vtk${KIT}Python vtk${KIT} ${KIT_PYTHON_LIBS})
    ENDIF(APPLE)
    IF(WIN32)
      TARGET_LINK_LIBRARIES (vtk${KIT}Python
        debug ${PYTHON_DEBUG_LIBRARY}
        optimized ${PYTHON_LIBRARY})
    ENDIF(WIN32)
    IF(WIN32 AND NOT CYGWIN)
      SET_TARGET_PROPERTIES(vtk${KIT}Python PROPERTIES SUFFIX ".pyd")
    ENDIF(WIN32 AND NOT CYGWIN)
    INSTALL_TARGETS(/lib/vtk vtk${KIT}Python)
  ENDIF(VTK_MAJOR_VERSION GREATER 4)
ENDIF (VTK_WRAP_PYTHON)

# Do the Java wrapping
#
IF (VTK_WRAP_JAVA)
  IF(VTK_MAJOR_VERSION GREATER 4)
    # Do the VTK 5 style wrapping
    #
    VTK_WRAP_JAVA3(vtk${KIT}Java KitJava_SRCS "${Kit_SRCS}")
    ADD_LIBRARY(vtk${KIT}Java SHARED ${KitJava_SRCS} ${Kit_JAVA_EXTRA_SRCS})
    SET(KIT_LIBRARY_TARGETS ${KIT_LIBRARY_TARGETS} vtk${KIT}Java)
    TARGET_LINK_LIBRARIES(vtk${KIT}Java vtk${KIT} ${KIT_JAVA_LIBS})
    IF(KIT_JAVA_DEPS)
      ADD_DEPENDENCIES(vtk${KIT}Java ${KIT_JAVA_DEPS})
    ENDIF(KIT_JAVA_DEPS)
  ELSE(VTK_MAJOR_VERSION GREATER 4)
    # Do the VTK 4 style wrapping
    #
    VTK_WRAP_JAVA2(vtk${KIT}Java KitJava_SRCS ${Kit_SRCS})
    ADD_LIBRARY(vtk${KIT}Java SHARED ${KitJava_SRCS} ${Kit_JAVA_EXTRA_SRCS})
    TARGET_LINK_LIBRARIES(vtk${KIT}Java vtk${KIT} ${KIT_JAVA_LIBS})
    INSTALL_TARGETS(/lib/vtk vtk${KIT}Java)
  ENDIF(VTK_MAJOR_VERSION GREATER 4)
ENDIF (VTK_WRAP_JAVA)

TARGET_LINK_LIBRARIES(vtk${KIT} ${KIT_LIBS} ${KIT_EXTRA_LIBS})

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
