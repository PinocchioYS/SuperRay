SET (gridmath2D_SRCS
  Vector2.cpp
  Pose3D.cpp
 )


ADD_LIBRARY( gridmath2D SHARED ${gridmath2D_SRCS})

SET_TARGET_PROPERTIES( gridmath2D PROPERTIES
  VERSION ${GRIDMAP2D_VERSION}
  SOVERSION ${GRIDMAP2D_SOVERSION}
  INSTALL_NAME_DIR ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}   # this seems to be necessary for MacOS X
)
# INSTALL_NAME_DIR seems to be necessary for MacOS X

ADD_LIBRARY( gridmath2D-static STATIC ${gridmath2D_SRCS})
SET_TARGET_PROPERTIES(gridmath2D-static PROPERTIES OUTPUT_NAME "gridmath2D")

if(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/gridmap2D")
    file(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/gridmap2D")
endif()

export(TARGETS gridmath2D gridmath2D-static
    APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/gridmap2D/gridmap2D-targets.cmake"
)

install(TARGETS gridmath2D gridmath2D-static
    EXPORT gridmap2D-targets
    INCLUDES DESTINATION include
    ${INSTALL_TARGETS_DEFAULT_ARGS}
)
 