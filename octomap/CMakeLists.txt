CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT( octomap )

include(CTest)
include(GNUInstallDirs)

# version (e.g. for packaging)
set(OCTOMAP_MAJOR_VERSION 1)
set(OCTOMAP_MINOR_VERSION 9)
set(OCTOMAP_PATCH_VERSION 7)
set(OCTOMAP_VERSION ${OCTOMAP_MAJOR_VERSION}.${OCTOMAP_MINOR_VERSION}.${OCTOMAP_PATCH_VERSION})
set(OCTOMAP_SOVERSION ${OCTOMAP_MAJOR_VERSION}.${OCTOMAP_MINOR_VERSION})
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
  if(POLICY CMP0042)
    # Enable MACOSX_RPATH by default.
    cmake_policy(SET CMP0042 NEW)
  endif(POLICY CMP0042)
endif(COMMAND cmake_policy)

SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CompilerSettings)

# OCTOMAP_OMP = enable OpenMP parallelization (experimental, defaults to OFF)
SET(OCTOMAP_OMP FALSE CACHE BOOL "Enable/disable OpenMP parallelization")
IF(DEFINED ENV{OCTOMAP_OMP})
  SET(OCTOMAP_OMP $ENV{OCTOMAP_OMP})
ENDIF(DEFINED ENV{OCTOMAP_OMP})
IF(OCTOMAP_OMP)
  FIND_PACKAGE( OpenMP REQUIRED)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF(OCTOMAP_OMP)

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
  LIBRARY DESTINATION lib/superray
  ARCHIVE DESTINATION lib/superray
)

ADD_SUBDIRECTORY( src/math )
ADD_SUBDIRECTORY( src )

file(GLOB octomap_HDRS ${PROJECT_SOURCE_DIR}/include/octomap/*.h ${PROJECT_SOURCE_DIR}/include/octomap/*.hxx)
install(FILES ${octomap_HDRS}	DESTINATION include/superray/octomap)
file(GLOB octomap_math_HDRS ${PROJECT_SOURCE_DIR}/include/octomap/math/*.h)
install(FILES ${octomap_math_HDRS}	DESTINATION include/superray/octomap/math)
file(GLOB octomap_superray_HDRS ${PROJECT_SOURCE_DIR}/include/octomap_superray/*.h)
install(FILES ${octomap_superray_HDRS}	DESTINATION include/superray/octomap_superray)
file(GLOB octomap_cullingregion_HDRS ${PROJECT_SOURCE_DIR}/include/octomap_cullingregion/*.h)
install(FILES ${octomap_cullingregion_HDRS}	DESTINATION include/superray/octomap_cullingregion)

# Install package.xml (catkin/ament/rosdep)
install(FILES package.xml DESTINATION share/superray_octomap)

# Allows Colcon to find non-Ament packages when using workspace underlays
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)

# uninstall target
configure_file(
    "${PROJECT_SOURCE_DIR}/CMakeModules/CMakeUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE superray_octomap)
 
# Create a octomap-config.cmake file for the use from the build tree
set(OCTOMAP_INCLUDE_DIRS "${INCLUDE_DIRS}")
set(OCTOMAP_LIB_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
# Set library names as absolute paths
# Windows, spec. MSVC requires the .lib suffix for imported libs
IF(WIN32)
  set(OCTOMAP_LIBRARY
    "${CMAKE_IMPORT_LIBRARY_PREFIX}octomap${CMAKE_IMPORT_LIBRARY_SUFFIX}"
  )
  set(OCTOMATH_LIBRARY
    "${CMAKE_IMPORT_LIBRARY_PREFIX}octomath${CMAKE_IMPORT_LIBRARY_SUFFIX}"
  )
ELSE()
  set(OCTOMAP_LIBRARY
    "${CMAKE_SHARED_LIBRARY_PREFIX}octomap${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )
  set(OCTOMATH_LIBRARY
    "${CMAKE_SHARED_LIBRARY_PREFIX}octomath${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )
ENDIF()

include(CMakePackageConfigHelpers)

CONFIGURE_PACKAGE_CONFIG_FILE(
  octomap-config.cmake.in
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap/octomap-config.cmake"
  PATH_VARS OCTOMAP_INCLUDE_DIRS OCTOMAP_LIB_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/octomap)

WRITE_BASIC_PACKAGE_VERSION_FILE(
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap/octomap-config-version.cmake"
  VERSION ${OCTOMAP_VERSION}
  COMPATIBILITY AnyNewerVersion)

# Create a superray_octomap-config.cmake file for the use from the install tree
# and install it
set(OCTOMAP_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include/superray")
set(OCTOMAP_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib/superray")
#set(OCTOMAP_CMAKE_DIR "${INSTALL_DATA_DIR}/FooBar/CMake")

CONFIGURE_PACKAGE_CONFIG_FILE(
  octomap-config.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/superray_octomap-config.cmake"
  PATH_VARS OCTOMAP_INCLUDE_DIRS OCTOMAP_LIB_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/superray_octomap)

WRITE_BASIC_PACKAGE_VERSION_FILE(
  ${PROJECT_BINARY_DIR}/InstallFiles/superray_octomap-config-version.cmake
  VERSION ${OCTOMAP_VERSION}
  COMPATIBILITY AnyNewerVersion)

install(FILES
  "${PROJECT_BINARY_DIR}/InstallFiles/superray_octomap-config.cmake"
  "${PROJECT_BINARY_DIR}/InstallFiles/superray_octomap-config-version.cmake"
  DESTINATION share/superray_octomap/)

# Write pkgconfig-file:
include(InstallPkgConfigFile)
install_pkg_config_file(octomap
    CFLAGS
    LIBS -loctomap -loctomath
    REQUIRES
    VERSION ${OCTOMAP_VERSION})

# Needs to be last statement:
INCLUDE(CPackSettings)

# Finished:
MESSAGE ("\n")
MESSAGE (STATUS "Compile octomap using: make")
MESSAGE (STATUS "Install octomap using: make install")
MESSAGE (STATUS "    (be sure to set the correct CMAKE_INSTALL_PREFIX before)")
