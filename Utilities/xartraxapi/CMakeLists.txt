PROJECT(VTKXARTRAXAPI)
INCLUDE_REGULAR_EXPRESSION("^(XarTraXAPI).*$")

INCLUDE_DIRECTORIES(${XARTRAXAPI_SOURCE_DIR})
INCLUDE_DIRECTORIES(${XARTRAXAPI_BINARY_DIR})

INCLUDE_DIRECTORIES(${VTK_SOURCE_DIR}/AtamaiTracking ${VTK_SOURCE_DIR}/AtamaiTracking/ndicapi)

SET(common_SRCS
${VTK_SOURCE_DIR}/AtamaiTracking/ndicapi/ndicapi_math.c ${VTK_SOURCE_DIR}/AtamaiTracking/ndicapi/ndicapi_serial.c ${VTK_SOURCE_DIR}/AtamaiTracking/ndicapi/ndicapi_thread.c XarTraXAPI.cpp)

IF (WIN32)
  IF (NOT BORLAND)
    IF (NOT CYGWIN)
      SET(common_SRCS ${common_SRCS} vtkxartraxapi.def)
    ENDIF (NOT CYGWIN)
  ENDIF (NOT BORLAND)
ENDIF (WIN32)

ADD_LIBRARY(vtkxartraxapi ${common_SRCS})
TARGET_LINK_LIBRARIES(vtkxartraxapi)

INSTALL_TARGETS(/lib/vtk vtkxartraxapi)
