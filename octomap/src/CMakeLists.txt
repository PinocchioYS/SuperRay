##### Sources ##########################################################################################################
SET(SUPERRAY_OCTOMAP_SOURCES
	SuperRayCloud.cpp
	SuperRayGenerator.cpp
	SuperRayOcTree.cpp
	CullingRegionOcTree.cpp
)

##### Library ##########################################################################################################
ADD_LIBRARY(superray-octomap SHARED ${SUPERRAY_OCTOMAP_SOURCES})
TARGET_LINK_LIBRARIES(superray-octomap octomap)

ADD_LIBRARY(superray-octomap-static STATIC ${SUPERRAY_OCTOMAP_SOURCES})
TARGET_LINK_LIBRARIES(superray-octomap-static octomap)

##### Execution ########################################################################################################
ADD_EXECUTABLE(example_superrayOcTree example_superrayOcTree.cpp)
TARGET_LINK_LIBRARIES(example_superrayOcTree superray-octomap octomap)

ADD_EXECUTABLE(example_cullingregionOcTree example_cullingregionOcTree.cpp)
TARGET_LINK_LIBRARIES(example_cullingregionOcTree superray-octomap octomap)

##### Install ##########################################################################################################
install(TARGETS
	superray-octomap
	superray-octomap-static
	example_superrayOcTree
	example_cullingregionOcTree
	${INSTALL_TARGETS_DEFAULT_ARGS}
)