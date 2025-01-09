##### Sources ##########################################################################################################
SET (SUPERRAY_GRIDMAP3D_SOURCES
  AbstractGrid3D.cpp
  AbstractOccupancyGrid3D.cpp
  Pointcloud.cpp
  ScanGraph.cpp
  Grid3D.cpp
  Grid3DNode.cpp
  SuperRayCloud.cpp
  SuperRayGenerator.cpp
  SuperRayGrid3D.cpp
  CullingRegionGrid3D.cpp
)

##### Library ##########################################################################################################
ADD_LIBRARY(superray-gridmap3d SHARED ${SUPERRAY_GRIDMAP3D_SOURCES})

ADD_LIBRARY(superray-gridmap3d-static STATIC ${SUPERRAY_GRIDMAP3D_SOURCES})
SET_TARGET_PROPERTIES(superray-gridmap3d-static PROPERTIES OUTPUT_NAME "superray-gridmap3d") 
ADD_DEPENDENCIES(superray-gridmap3d-static superray-gridmath3d-static)

TARGET_LINK_LIBRARIES(superray-gridmap3d superray-gridmath3d)

##### Execution ########################################################################################################
ADD_EXECUTABLE(example_superrayGrid3D example_superrayGrid3D.cpp)
TARGET_LINK_LIBRARIES(example_superrayGrid3D superray-gridmap3d)

ADD_EXECUTABLE(example_cullingregionGrid3D example_cullingregionGrid3D.cpp)
TARGET_LINK_LIBRARIES(example_cullingregionGrid3D superray-gridmap3d)

##### Install ##########################################################################################################
IF(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-gridmap3d")
  FILE(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-gridmap3d")
ENDIF()

EXPORT(TARGETS superray-gridmap3d superray-gridmap3d-static
  APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-gridmap3d/superray-gridmap3d-targets.cmake"
)

INSTALL(TARGETS 
  superray-gridmap3d
  superray-gridmap3d-static 
  example_superrayGrid3D
  example_cullingregionGrid3D
  ${INSTALL_TARGETS_DEFAULT_ARGS}
)