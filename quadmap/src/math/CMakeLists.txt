SET (quadmath_SRCS
  Vector2.cpp
#  Quaternion.cpp
  Pose3D.cpp
 )


ADD_LIBRARY( quadmath SHARED ${quadmath_SRCS})

SET_TARGET_PROPERTIES( quadmath PROPERTIES
  VERSION ${QUADMAP_VERSION}
  SOVERSION ${QUADMAP_SOVERSION}
  INSTALL_NAME_DIR ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}   # this seems to be necessary for MacOS X
)
# INSTALL_NAME_DIR seems to be necessary for MacOS X

ADD_LIBRARY( quadmath-static STATIC ${quadmath_SRCS})
SET_TARGET_PROPERTIES(quadmath-static PROPERTIES OUTPUT_NAME "quadmath")

install(TARGETS quadmath quadmath-static ${INSTALL_TARGETS_DEFAULT_ARGS})
 