SET( superray_SRCS
   SuperRayOcTree.cpp
   SuperRayGenerator.cpp
   SuperRayCloud.cpp
   )

add_library(superray SHARED ${superray_SRCS})
set_target_properties(superray PROPERTIES
  VERSION ${SUPERRAY_VERSION}
  SOVERSION ${SUPERRAY_SOVERSION}
)
target_link_libraries(superray ${OCTOMAP_LIBRARIES})

add_library(superray-static STATIC ${superray_SRCS})
target_link_libraries(superray-static ${OCTOMAP_LIBRARIES})

SET_TARGET_PROPERTIES(superray-static PROPERTIES OUTPUT_NAME "superray") 

# directly depend on the octomap library target when building the
# complete distribution, so it it recompiled as needed
if (CMAKE_PROJECT_NAME STREQUAL "superray-distribution")
  ADD_DEPENDENCIES(superray-static octomap-static)
  ADD_DEPENDENCIES(superray octomap)
endif()

ADD_SUBDIRECTORY(examples)

install(TARGETS 
  superray
  superray-static
  ${INSTALL_TARGETS_DEFAULT_ARGS})
