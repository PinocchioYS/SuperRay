##### Sources ##########################################################################################################
SET (SUPERRAY_GRIDMATH2D_SOURCES
  Vector2.cpp
  Pose3D.cpp
)

##### Library ##########################################################################################################
ADD_LIBRARY(superray-gridmath2d SHARED ${SUPERRAY_GRIDMATH2D_SOURCES})

ADD_LIBRARY(superray-gridmath2d-static STATIC ${SUPERRAY_GRIDMATH2D_SOURCES})
SET_TARGET_PROPERTIES(superray-gridmath2d-static PROPERTIES OUTPUT_NAME "superray-gridmath2d")

IF(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-gridmap2d")
  FILE(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-gridmap2d")
ENDIF()

EXPORT(TARGETS superray-gridmath2d superray-gridmath2d-static
  APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-gridmap2d/superray-gridmap2d-targets.cmake"
)

##### Install ##########################################################################################################
INSTALL(TARGETS superray-gridmath2d superray-gridmath2d-static
  EXPORT superray-gridmap2d-targets
  INCLUDES DESTINATION include
  ${INSTALL_TARGETS_DEFAULT_ARGS}
)