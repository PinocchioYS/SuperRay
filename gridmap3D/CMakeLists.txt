CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
PROJECT( gridmap3D )

ENABLE_TESTING()

# version (e.g. for packaging)
set(GRIDMAP3D_MAJOR_VERSION 1)
set(GRIDMAP3D_MINOR_VERSION 0)
set(GRIDMAP3D_PATCH_VERSION 0)
set(GRIDMAP3D_VERSION ${GRIDMAP3D_MAJOR_VERSION}.${GRIDMAP3D_MINOR_VERSION}.${GRIDMAP3D_PATCH_VERSION})
set(GRIDMAP3D_SOVERSION ${GRIDMAP3D_MAJOR_VERSION}.${GRIDMAP3D_MINOR_VERSION})
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CompilerSettings)

# GRIDMAP3D_OMP = enable OpenMP parallelization (experimental, defaults to OFF)
SET(GRIDMAP3D_OMP FALSE CACHE BOOL "Enable/disable OpenMP parallelization")
IF(DEFINED ENV{GRIDMAP3D_OMP})
  SET(GRIDMAP3D_OMP $ENV{GRIDMAP3D_OMP})
ENDIF(DEFINED ENV{GRIDMAP3D_OMP})
IF(GRIDMAP3D_OMP)
  FIND_PACKAGE( OpenMP REQUIRED)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF(GRIDMAP3D_OMP)

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

ADD_SUBDIRECTORY( src/math )
ADD_SUBDIRECTORY( src )

file(GLOB gridmap3D_HDRS ${PROJECT_SOURCE_DIR}/include/gridmap3D/*.h ${PROJECT_SOURCE_DIR}/include/gridmap3D/*.hxx)
install(FILES ${gridmap3D_HDRS}	DESTINATION include/gridmap3D)
file(GLOB gridmap3D_math_HDRS ${PROJECT_SOURCE_DIR}/include/gridmap3D/math/*.h)
install(FILES ${gridmap3D_math_HDRS}	DESTINATION include/gridmap3D/math)

# Install catkin package.xml
install(FILES package.xml DESTINATION share/gridmap3D)

# uninstall target
#configure_file(
#    "${PROJECT_SOURCE_DIR}/CMakeModules/CMakeUninstall.cmake.in"
#    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
#    IMMEDIATE @ONLY)

#add_custom_target(uninstall
#    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE gridmap3D)
 
# Create a gridmap3D-config.cmake file for the use from the build tree
set(GRIDMAP3D_INCLUDE_DIRS "${INCLUDE_DIRS}")
set(GRIDMAP3D_LIB_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
# Set library names as absolute paths
# Windows, spec. MSVC requires the .lib suffix for imported libs
IF(WIN32)
  set(GRIDMAP3D_LIBRARY
    "${CMAKE_IMPORT_LIBRARY_PREFIX}gridmap3d${CMAKE_IMPORT_LIBRARY_SUFFIX}"
  )
  set(GRIDMATH3D_LIBRARY
    "${CMAKE_IMPORT_LIBRARY_PREFIX}gridmath3d${CMAKE_IMPORT_LIBRARY_SUFFIX}"
  )
ELSE()
  set(GRIDMAP3D_LIBRARY
    "${CMAKE_SHARED_LIBRARY_PREFIX}gridmap3d${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )
  set(GRIDMATH3D_LIBRARY
    "${CMAKE_SHARED_LIBRARY_PREFIX}gridmath3d${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )
ENDIF()

# not used right now (export depends?)
#set(GRIDMAP3D_CMAKE_DIR "${PROJECT_BINARY_DIR}")
include(CMakePackageConfigHelpers)

CONFIGURE_PACKAGE_CONFIG_FILE(
  gridmap3D-config.cmake.in
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/gridmap3D/gridmap3d-config.cmake"
  PATH_VARS GRIDMAP3D_INCLUDE_DIRS GRIDMAP3D_LIB_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gridmap3D)

WRITE_BASIC_PACKAGE_VERSION_FILE(
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/gridmap3D/gridmap3d-config-version.cmake"
  VERSION ${GRIDMAP3D_VERSION}
  COMPATIBILITY AnyNewerVersion)

# Create a gridmap3D-config.cmake file for the use from the install tree
# and install it
set(GRIDMAP3D_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(GRIDMAP3D_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")
#set(GRIDMAP3D_CMAKE_DIR "${INSTALL_DATA_DIR}/FooBar/CMake")

CONFIGURE_PACKAGE_CONFIG_FILE(
  gridmap3D-config.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/gridmap3d-config.cmake"
  PATH_VARS GRIDMAP3D_INCLUDE_DIRS GRIDMAP3D_LIB_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gridmap3D)

WRITE_BASIC_PACKAGE_VERSION_FILE(
  ${PROJECT_BINARY_DIR}/InstallFiles/gridmap3d-config-version.cmake
  VERSION ${GRIDMAP3D_VERSION}
  COMPATIBILITY AnyNewerVersion)

install(FILES
  "${PROJECT_BINARY_DIR}/InstallFiles/gridmap3d-config.cmake"
  "${PROJECT_BINARY_DIR}/InstallFiles/gridmap3d-config-version.cmake"
  DESTINATION share/gridmap3D/)

# Write pkgconfig-file:
include(InstallPkgConfigFile)
install_pkg_config_file(gridmap3D
    CFLAGS
    LIBS -lgridmap3D -lgridmath3D
    REQUIRES
    VERSION ${GRIDMAP3D_VERSION})

# Needs to be last statement:
INCLUDE(CPackSettings)

# Finished:
MESSAGE ("\n")
MESSAGE (STATUS "Compile gridmap3D using: make")
MESSAGE (STATUS "Install gridmap3D using: make install")
MESSAGE (STATUS "    (be sure to set the correct CMAKE_INSTALL_PREFIX before)")
