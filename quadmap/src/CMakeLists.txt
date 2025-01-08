##### Sources ##########################################################################################################
SET(SUPERRAY_QUADMAP_SOURCES
  AbstractQuadTree.cpp
  AbstractOccupancyQuadTree.cpp
  Pointcloud.cpp
  ScanGraph.cpp
  QuadTree.cpp
  QuadTreeNode.cpp
  SuperRayCloud.cpp
  SuperRayGenerator.cpp
  SuperRayQuadTree.cpp
  CullingRegionQuadTree.cpp
)

##### Library ##########################################################################################################
ADD_LIBRARY(superray-quadmap SHARED ${SUPERRAY_QUADMAP_SOURCES})

ADD_LIBRARY(superray-quadmap-static STATIC ${SUPERRAY_QUADMAP_SOURCES})
SET_TARGET_PROPERTIES(superray-quadmap-static PROPERTIES OUTPUT_NAME "superray-quadmap")
ADD_DEPENDENCIES(superray-quadmap-static superray-quadmath-static)

TARGET_LINK_LIBRARIES(superray-quadmap superray-quadmath)

##### Execution ########################################################################################################
ADD_EXECUTABLE(example_superrayQuadTree example_superrayQuadTree.cpp)
TARGET_LINK_LIBRARIES(example_superrayQuadTree superray-quadmap)

ADD_EXECUTABLE(example_cullingregionQuadTree example_cullingregionQuadTree.cpp)
TARGET_LINK_LIBRARIES(example_cullingregionQuadTree superray-quadmap)

##### Install ##########################################################################################################
IF(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-quadmap")
  FILE(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-quadmap")
ENDIF()

EXPORT(TARGETS superray-quadmap superray-quadmap-static
  APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-quadmap/superray-quadmap-targets.cmake")

INSTALL(TARGETS 
	superray-quadmap
	superray-quadmap-static
	example_superrayQuadTree
  example_cullingregionQuadTree
	${INSTALL_TARGETS_DEFAULT_ARGS}
)

