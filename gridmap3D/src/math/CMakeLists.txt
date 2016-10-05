SET (gridmath3D_SRCS
  Vector3.cpp
  Quaternion.cpp
  Pose6D.cpp
 )


ADD_LIBRARY( gridmath3D SHARED ${gridmath3D_SRCS})

SET_TARGET_PROPERTIES( gridmath3D PROPERTIES
  VERSION ${GRIDMAP3D_VERSION}
  SOVERSION ${GRIDMAP3D_SOVERSION}
  INSTALL_NAME_DIR ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}   # this seems to be necessary for MacOS X
)
# INSTALL_NAME_DIR seems to be necessary for MacOS X

ADD_LIBRARY( gridmath3D-static STATIC ${gridmath3D_SRCS})
SET_TARGET_PROPERTIES(gridmath3D-static PROPERTIES OUTPUT_NAME "gridmath3D")

install(TARGETS gridmath3D gridmath3D-static ${INSTALL_TARGETS_DEFAULT_ARGS})
 