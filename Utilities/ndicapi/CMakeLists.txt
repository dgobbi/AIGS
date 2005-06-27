PROJECT(VTKNDICAPI)
INCLUDE_REGULAR_EXPRESSION("^(ndicapi|polaris).*$")

INCLUDE_DIRECTORIES(${VTKNDICAPI_SOURCE_DIR})
INCLUDE_DIRECTORIES(${VTKNDICAPI_BINARY_DIR})

SET(common_SRCS
ndicapi.c ndicapi_math.c ndicapi_serial.c ndicapi_thread.c)

IF (WIN32)
  IF (NOT BORLAND)
    IF (NOT CYGWIN)
      SET(common_SRCS ${common_SRCS} vtkndicapi.def)
    ENDIF (NOT CYGWIN)
  ENDIF (NOT BORLAND)
ENDIF (WIN32)

ADD_LIBRARY(vtkndicapi ${common_SRCS})
TARGET_LINK_LIBRARIES( vtkndicapi )

INSTALL_TARGETS(/lib/vtk vtkndicapi)
