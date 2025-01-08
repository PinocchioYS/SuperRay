##### Sources ##########################################################################################################
SET (SUPERRAY_QUADMATH_SOURCES
  Vector2.cpp
  Pose3D.cpp
)

##### Library ##########################################################################################################
ADD_LIBRARY(superray-quadmath SHARED ${SUPERRAY_QUADMATH_SOURCES})

ADD_LIBRARY(superray-quadmath-static STATIC ${SUPERRAY_QUADMATH_SOURCES})
SET_TARGET_PROPERTIES(superray-quadmath-static PROPERTIES OUTPUT_NAME "superray-quadmath")

IF(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-quadmap")
  FILE(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-quadmap")
ENDIF()

EXPORT(TARGETS superray-quadmath superray-quadmath-static
  APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-quadmap/superray-quadmap-targets.cmake"
)

##### Install ##########################################################################################################
INSTALL(TARGETS superray-quadmath superray-quadmath-static
  EXPORT superray-quadmap-targets
  INCLUDES DESTINATION include
  ${INSTALL_TARGETS_DEFAULT_ARGS}
)
 