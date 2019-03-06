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
)

# dynamic and static libs, see CMake FAQ:
ADD_LIBRARY( octomap SHARED ${octomap_SRCS})
set_target_properties( octomap PROPERTIES
	VERSION ${OCTOMAP_VERSION}
	SOVERSION ${OCTOMAP_SOVERSION}
)

ADD_LIBRARY( octomap-static STATIC ${octomap_SRCS})
SET_TARGET_PROPERTIES(octomap-static PROPERTIES OUTPUT_NAME "octomap")

TARGET_LINK_LIBRARIES(octomap octomath)

ADD_EXECUTABLE(example_superrayOcTree example_superrayOcTree.cpp)
TARGET_LINK_LIBRARIES(example_superrayOcTree octomap)

install(TARGETS
	octomap
	octomap-static
	example_superrayOcTree
	${INSTALL_TARGETS_DEFAULT_ARGS}
)

