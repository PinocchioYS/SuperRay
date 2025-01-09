##### Sources ##########################################################################################################
SET (SUPERRAY_GRIDMAP2D_SOURCES
  AbstractGrid2D.cpp
  AbstractOccupancyGrid2D.cpp
  Pointcloud.cpp
  ScanGraph.cpp
  Grid2D.cpp
  Grid2DNode.cpp
  SuperRayCloud.cpp
  SuperRayGenerator.cpp
  SuperRayGrid2D.cpp
  CullingRegionGrid2D.cpp
)

##### Library ##########################################################################################################
ADD_LIBRARY(superray-gridmap2d SHARED ${SUPERRAY_GRIDMAP2D_SOURCES})

ADD_LIBRARY(superray-gridmap2d-static STATIC ${SUPERRAY_GRIDMAP2D_SOURCES})
SET_TARGET_PROPERTIES(superray-gridmap2d-static PROPERTIES OUTPUT_NAME "superray-gridmap2d") 
ADD_DEPENDENCIES(superray-gridmap2d-static superray-gridmath2d-static)

TARGET_LINK_LIBRARIES(superray-gridmap2d superray-gridmath2d)

##### Execution ########################################################################################################
ADD_EXECUTABLE(example_superrayGrid2D example_superrayGrid2D.cpp)
TARGET_LINK_LIBRARIES(example_superrayGrid2D superray-gridmap2d)

ADD_EXECUTABLE(example_cullingregionGrid2D example_cullingregionGrid2D.cpp)
TARGET_LINK_LIBRARIES(example_cullingregionGrid2D superray-gridmap2d)

##### Install ##########################################################################################################
IF(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-gridmap2d")
  FILE(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-gridmap2d")
ENDIF()

EXPORT(TARGETS superray-gridmap2d superray-gridmap2d-static
  APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-gridmap2d/superray-gridmap2d-targets.cmake"
)

INSTALL(TARGETS 
	superray-gridmap2d
	superray-gridmap2d-static 
	example_superrayGrid2D
  example_cullingregionGrid2D
	${INSTALL_TARGETS_DEFAULT_ARGS}
)