SET (quadmath_SRCS
  Vector2.cpp
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

if(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/quadmap")
    file(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/quadmap")
endif()

export(TARGETS quadmath quadmath-static
    APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/quadmap/quadmap-targets.cmake"
)

install(TARGETS quadmath quadmath-static
    EXPORT quadmap-targets
    INCLUDES DESTINATION include
    ${INSTALL_TARGETS_DEFAULT_ARGS}
)
 