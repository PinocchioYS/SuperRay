SET (gridmap2D_SRCS
  AbstractGrid2D.cpp
  AbstractOccupancyGrid2D.cpp
  Pointcloud.cpp
  ScanGraph.cpp
  Grid2D.cpp
  Grid2DNode.cpp
  SuperRayCloud.cpp
  SuperRayGenerator.cpp
  SuperRayGrid2D.cpp
  )

# dynamic and static libs, see CMake FAQ:
ADD_LIBRARY( gridmap2D SHARED ${gridmap2D_SRCS})
set_target_properties( gridmap2D PROPERTIES
  VERSION ${GRIDMAP2D_VERSION}
  SOVERSION ${GRIDMAP2D_SOVERSION}
)
ADD_LIBRARY( gridmap2D-static STATIC ${gridmap2D_SRCS})
SET_TARGET_PROPERTIES(gridmap2D-static PROPERTIES OUTPUT_NAME "gridmap2D") 

TARGET_LINK_LIBRARIES(gridmap2D gridmath2D)

ADD_EXECUTABLE(example_superrayGrid2D example_superrayGrid2D.cpp)
TARGET_LINK_LIBRARIES(example_superrayGrid2D gridmap2D)

install(TARGETS 
	gridmap2D
	gridmap2D-static 
	example_superrayGrid2D
	${INSTALL_TARGETS_DEFAULT_ARGS}
)

