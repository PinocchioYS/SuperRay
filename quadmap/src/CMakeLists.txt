SET (quadmap_SRCS
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

# dynamic and static libs, see CMake FAQ:
ADD_LIBRARY( quadmap SHARED ${quadmap_SRCS})
set_target_properties( quadmap PROPERTIES
  VERSION ${QUADMAP_VERSION}
  SOVERSION ${QUADMAP_SOVERSION}
)
ADD_LIBRARY( quadmap-static STATIC ${quadmap_SRCS})
SET_TARGET_PROPERTIES(quadmap-static PROPERTIES OUTPUT_NAME "quadmap")

TARGET_LINK_LIBRARIES(quadmap quadmath)

ADD_EXECUTABLE(example_superrayQuadTree example_superrayQuadTree.cpp)
TARGET_LINK_LIBRARIES(example_superrayQuadTree quadmap)

ADD_EXECUTABLE(example_cullingregionQuadTree example_cullingregionQuadTree.cpp)
TARGET_LINK_LIBRARIES(example_cullingregionQuadTree quadmap)

install(TARGETS 
	quadmap
	quadmap-static
	example_superrayQuadTree
    example_cullingregionQuadTree
	${INSTALL_TARGETS_DEFAULT_ARGS}
)

