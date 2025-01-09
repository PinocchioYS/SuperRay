CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT( superray-conversion )

INCLUDE(GNUInstallDirs)

# COMPILER SETTINGS (default: Release)
# use "-DCMAKE_BUILD_TYPE=Debug" in cmake for a Debug-build
IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

# COMPILER FLAGS
IF (CMAKE_COMPILER_IS_GNUCC)
  SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-error ")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -Wpedantic")
  SET (CMAKE_CXX_FLAGS_RELEASE "-O3 -funroll-loops -DNDEBUG")
  SET (CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
  # Shared object compilation under 64bit (vtable)
  ADD_DEFINITIONS(-fPIC)
ENDIF()

# Set rpath
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FIND_PACKAGE(octomap REQUIRED)
FIND_PACKAGE(superray-quadmap REQUIRED HINTS ${CMAKE_SOURCE_DIR}/lib/cmake/superray-quadmap)

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )
# output dirs for multi-config builds (MSVC)
FOREACH( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
  STRING( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
  SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${BASE_DIR}/lib )
  SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${BASE_DIR}/lib )
  SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${BASE_DIR}/bin )
ENDFOREACH( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )

INCLUDE_DIRECTORIES(
  # include
  ${OCTOMAP_INCLUDE_DIRS}
  ${SUPERRAY_QUADMAP_INCLUDE_DIRS}
)

##### Execution ########################################################################################################
ADD_EXECUTABLE(scan_graph_projection
  src/scan_graph_projection.cpp
)
TARGET_LINK_LIBRARIES(scan_graph_projection
  ${OCTOMAP_LIBRARIES}
	${SUPERRAY_QUADMAP_LIBRARIES}
)

##### Install ##########################################################################################################
INSTALL(TARGETS
  scan_graph_projection
	${INSTALL_TARGETS_DEFAULT_ARGS}
)