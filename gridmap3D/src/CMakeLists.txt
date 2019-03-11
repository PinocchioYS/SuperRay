SET (gridmap3D_SRCS
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

# dynamic and static libs, see CMake FAQ:
ADD_LIBRARY( gridmap3D SHARED ${gridmap3D_SRCS})
set_target_properties( gridmap3D PROPERTIES
    VERSION ${GRIDMAP3D_VERSION}
    SOVERSION ${GRIDMAP3D_SOVERSION}
)

ADD_LIBRARY( gridmap3D-static STATIC ${gridmap3D_SRCS})
SET_TARGET_PROPERTIES(gridmap3D-static PROPERTIES OUTPUT_NAME "gridmap3D") 

TARGET_LINK_LIBRARIES(gridmap3D gridmath3D)

ADD_EXECUTABLE(example_superrayGrid3D example_superrayGrid3D.cpp)
TARGET_LINK_LIBRARIES(example_superrayGrid3D gridmap3D)

ADD_EXECUTABLE(example_cullingregionGrid3D example_cullingregionGrid3D.cpp)
TARGET_LINK_LIBRARIES(example_cullingregionGrid3D gridmap3D)

install(TARGETS 
	gridmap3D
	gridmap3D-static
	example_superrayGrid3D
    example_cullingregionGrid3D
	${INSTALL_TARGETS_DEFAULT_ARGS}
)

