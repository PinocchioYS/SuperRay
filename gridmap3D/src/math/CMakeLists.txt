##### Sources ##########################################################################################################
SET (SUPERRAY_GRIDMATH3D_SOURCES
  Vector3.cpp
  Quaternion.cpp
  Pose6D.cpp
)

##### Library ##########################################################################################################
ADD_LIBRARY(superray-gridmath3d SHARED ${SUPERRAY_GRIDMATH3D_SOURCES})

ADD_LIBRARY(superray-gridmath3d-static STATIC ${SUPERRAY_GRIDMATH3D_SOURCES})
SET_TARGET_PROPERTIES(superray-gridmath3d-static PROPERTIES OUTPUT_NAME "superray-gridmath3d")

IF(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-gridmap3d")
  FILE(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-gridmap3d")
ENDIF()

EXPORT(TARGETS superray-gridmath3d superray-gridmath3d-static
  APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-gridmap3d/superray-gridmap3d-targets.cmake"
)

##### Install ##########################################################################################################
INSTALL(TARGETS superray-gridmath3d superray-gridmath3d-static
  EXPORT superray-gridmap3d-targets
  INCLUDES DESTINATION include
  ${INSTALL_TARGETS_DEFAULT_ARGS}
)