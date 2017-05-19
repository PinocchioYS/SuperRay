CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
PROJECT( superray )

ENABLE_TESTING()

# version (e.g. for packaging)
set(SUPERRAY_MAJOR_VERSION 1)
set(SUPERRAY_MINOR_VERSION 0)
set(SUPERRAY_PATCH_VERSION 0)
set(SUPERRAY_VERSION ${SUPERRAY_MAJOR_VERSION}.${SUPERRAY_MINOR_VERSION}.${SUPERRAY_PATCH_VERSION})
set(SUPERRAY_SOVERSION ${SUPERRAY_MAJOR_VERSION}.${SUPERRAY_MINOR_VERSION})

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CompilerSettings)

# SUPERRAY_OMP = enable OpenMP parallelization (defaults to OFF)
SET(SUPERRAY_OMP FALSE CACHE BOOL "Enable/disable OpenMP parallelization")
IF(DEFINED ENV{SUPERRAY_OMP})
  SET(SUPERRAY_OMP $ENV{SUPERRAY_OMP})
ENDIF(DEFINED ENV{SUPERRAY_OMP})
IF(SUPERRAY_OMP)
  FIND_PACKAGE( OpenMP REQUIRED)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF(SUPERRAY_OMP)

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )
# output dirs for multi-config builds (MSVC)
foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
  STRING( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
  SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${BASE_DIR}/lib )
  SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${BASE_DIR}/lib )
  SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${BASE_DIR}/bin )
endforeach( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )

set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

LINK_DIRECTORIES(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

# Installation

set(INSTALL_TARGETS_DEFAULT_ARGS
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

# Dependency of OctoMap Library
find_package(octomap REQUIRED
  HINTS ${CMAKE_SOURCE_DIR}/lib/cmake/octomap
   ${CMAKE_SOURCE_DIR}/../octomap/lib/cmake/octomap
)
MESSAGE(STATUS "Found octomap version: " ${octomap_VERSION})
MESSAGE(STATUS "octomap libraries: ${OCTOMAP_LIBRARIES}")

INCLUDE_DIRECTORIES(${OCTOMAP_INCLUDE_DIRS})

# Dependency of QuadMap
find_package(quadmap REQUIRED
  HINTS ${CMAKE_SOURCE_DIR}/lib/cmake/quadmap
   ${CMAKE_SOURCE_DIR}/../quadmap/lib/cmake/quadmap
)
MESSAGE(STATUS "Found quadmap version: " ${quadmap_VERSION})
MESSAGE(STATUS "quadmap libraries: ${QUADMAP_LIBRARIES}")

INCLUDE_DIRECTORIES(${QUADMAP_INCLUDE_DIRS})

# Dependency of GridMap3D
find_package(gridmap3D REQUIRED
  HINTS ${CMAKE_SOURCE_DIR}/lib/cmake/gridmap3D
   ${CMAKE_SOURCE_DIR}/../gridmap3D/lib/cmake/gridmap3D
)
MESSAGE(STATUS "Found gridmap3D version: " ${gridmap3D_VERSION})
MESSAGE(STATUS "gridmap3D libraries: ${GRIDMAP3D_LIBRARIES}")

INCLUDE_DIRECTORIES(${GRIDMAP3D_INCLUDE_DIRS})

# Dependency of GridMap2D
find_package(gridmap2D REQUIRED
  HINTS ${CMAKE_SOURCE_DIR}/lib/cmake/gridmap2D
   ${CMAKE_SOURCE_DIR}/../gridmap2D/lib/cmake/gridmap2D
)
MESSAGE(STATUS "Found gridmap2D version: " ${gridmap2D_VERSION})
MESSAGE(STATUS "gridmap2D libraries: ${GRIDMAP2D_LIBRARIES}")

INCLUDE_DIRECTORIES(${QUADMAP_INCLUDE_DIRS})

ADD_SUBDIRECTORY(src)

file(GLOB superray_HDRS ${PROJECT_SOURCE_DIR}/include/superray/*.h ${PROJECT_SOURCE_DIR}/include/superray/*.hxx)
install(FILES ${superray_HDRS}	DESTINATION include/superray)

# Install catkin package.xml, attention package.xml names the catkin package "superray", so this is also the location where it needs to be installed to (and not "superray")
install(FILES package.xml DESTINATION share/superray)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE superray)
 
# Create a dynamicEDT3DConfig.cmake file for the use from the build tree
set(SUPERRAY_INCLUDE_DIRS "${INCLUDE_DIRS}")
set(SUPERRAY_LIB_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
# Set library names as absolute paths
# Windows, spec. MSVC requires the .lib suffix for imported libs
IF(WIN32)
  set(SUPERRAY_LIBRARY
    "${CMAKE_IMPORT_LIBRARY_PREFIX}superray${CMAKE_IMPORT_LIBRARY_SUFFIX}"
  )
ELSE()
  set(SUPERRAY_LIBRARY
    "${CMAKE_SHARED_LIBRARY_PREFIX}superray${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )
ENDIF()
# not used right now (export depends?)
#set(DYNEDT3D_CMAKE_DIR "${PROJECT_BINARY_DIR}")
include(CMakePackageConfigHelpers)

CONFIGURE_PACKAGE_CONFIG_FILE(
  superrayConfig.cmake.in
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray/superrayConfig.cmake"
  PATH_VARS SUPERRAY_INCLUDE_DIRS SUPERRAY_LIB_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/superray)

WRITE_BASIC_PACKAGE_VERSION_FILE(
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray/superrayConfig-version.cmake"
  VERSION ${SUPERRAY_VERSION}
  COMPATIBILITY AnyNewerVersion)

# Install the export set for use with the install-tree
#install(EXPORT FooBarLibraryDepends DESTINATION
#  "${INSTALL_DATA_DIR}/FooBar/CMake"
#  COMPONENT dev)
 
# Create a dynamicEDT3DConfig.cmake file for the use from the install tree
# and install it
set(SUPERRAY_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(SUPERRAY_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")
#set(SUPERRAY_CMAKE_DIR "${INSTALL_DATA_DIR}/FooBar/CMake")

CONFIGURE_PACKAGE_CONFIG_FILE(
  superrayConfig.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/superrayConfig.cmake"
  PATH_VARS SUPERRAY_INCLUDE_DIRS SUPERRAY_LIB_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/superray)

WRITE_BASIC_PACKAGE_VERSION_FILE(
  "${PROJECT_BINARY_DIR}/InstallFiles/superrayConfig-version.cmake"
  VERSION ${SUPERRAY_VERSION}
  COMPATIBILITY AnyNewerVersion)

install(FILES
  "${PROJECT_BINARY_DIR}/InstallFiles/superrayConfig.cmake"
  "${PROJECT_BINARY_DIR}/InstallFiles/superrayConfig-version.cmake" 
  DESTINATION share/superray/)

# Write pkgconfig-file:
include(InstallPkgConfigFile)
install_pkg_config_file(superray
    CFLAGS
    LIBS -lsuperray
    REQUIRES
    VERSION ${SUPERRAY_VERSION})

# Needs to be last statement:
INCLUDE(CPackSettings)

# Finished:
MESSAGE ("\n")
MESSAGE (STATUS "Compile superray using: make")
MESSAGE (STATUS "Install superray using: make install")
MESSAGE (STATUS "    (be sure to set the correct CMAKE_INSTALL_PREFIX before)")
