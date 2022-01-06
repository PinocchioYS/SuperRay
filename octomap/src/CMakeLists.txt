SET (octomap_SRCS
	AbstractOcTree.cpp
	AbstractOccupancyOcTree.cpp
	Pointcloud.cpp
	ScanGraph.cpp
	CountingOcTree.cpp
	OcTree.cpp
	OcTreeNode.cpp
	OcTreeStamped.cpp
	ColorOcTree.cpp
	SuperRayCloud.cpp
	SuperRayGenerator.cpp
	SuperRayOcTree.cpp
	CullingRegionOcTree.cpp
)

# dynamic and static libs, see CMake FAQ:
ADD_LIBRARY( octomap SHARED ${octomap_SRCS})
set_target_properties( octomap PROPERTIES
	VERSION ${OCTOMAP_VERSION}
	SOVERSION ${OCTOMAP_SOVERSION}
)

ADD_LIBRARY( octomap-static STATIC ${octomap_SRCS})
SET_TARGET_PROPERTIES(octomap-static PROPERTIES OUTPUT_NAME "octomap") 
add_dependencies(octomap-static octomath-static)

TARGET_LINK_LIBRARIES(octomap octomath)

ADD_EXECUTABLE(example_superrayOcTree example_superrayOcTree.cpp)
TARGET_LINK_LIBRARIES(example_superrayOcTree octomap)

ADD_EXECUTABLE(example_cullingregionOcTree example_cullingregionOcTree.cpp)
TARGET_LINK_LIBRARIES(example_cullingregionOcTree octomap)

install(TARGETS
	octomap
	octomap-static
	example_superrayOcTree
	example_cullingregionOcTree
	${INSTALL_TARGETS_DEFAULT_ARGS}
)

