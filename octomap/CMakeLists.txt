CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT( superray-octomap )

INCLUDE(GNUInstallDirs)

# Dependency: OctoMap library
FIND_PACKAGE(octomap REQUIRED)

# COMPILER SETTINGS (default: Release) and flags
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")
INCLUDE(CompilerSettings)

# SUPERRAY_OMP = enable OpenMP parallelization (defaults to OFF)
SET(SUPERRAY_OMP FALSE CACHE BOOL "Enable/disable OpenMP parallelization")
IF(DEFINED ENV{SUPERRAY_OMP})
  SET(SUPERRAY_OMP $ENV{SUPERRAY_OMP})
ENDIF(DEFINED ENV{SUPERRAY_OMP})
IF(SUPERRAY_OMP)
  FIND_PACKAGE(OpenMP REQUIRED)
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
FOREACH( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
  STRING( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
  SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${BASE_DIR}/lib )
  SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${BASE_DIR}/lib )
  SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${BASE_DIR}/bin )
ENDFOREACH( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )

SET(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES(
  ${INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
  ${OCTOMAP_LIBRARY_DIRS}
)

SET(INSTALL_TARGETS_DEFAULT_ARGS
	RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
	LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
	ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)

ADD_SUBDIRECTORY( src )

FILE(GLOB superray_octomap_HDRS ${PROJECT_SOURCE_DIR}/include/superray_octomap/*.h)
INSTALL(FILES ${superray_octomap_HDRS}	DESTINATION include/superray/superray_octomap)
FILE(GLOB cullingregion_octomap_HDRS ${PROJECT_SOURCE_DIR}/include/cullingregion_octomap/*.h)
INSTALL(FILES ${cullingregion_octomap_HDRS}	DESTINATION include/superray/cullingregion_octomap)

# Install package.xml (catkin/ament/rosdep)
INSTALL(FILES package.xml DESTINATION "${CMAKE_INSTALL_DATADIR}/superray-octomap")

# Allows Colcon to find non-Ament packages when using workspace underlays
FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)

# uninstall target
IF(NOT TARGET uninstall)
  CONFIGURE_FILE(
    "${PROJECT_SOURCE_DIR}/CMakeModules/CMakeUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY
  )

  ADD_CUSTOM_TARGET(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
  )
ENDIF()

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
EXPORT(PACKAGE superray-octomap)

# Create a superray-octomap-config.cmake file for the use from the build tree
SET(SUPERRAY_OCTOMAP_INCLUDE_DIRS "${INCLUDE_DIRS}/superray")
SET(SUPERRAY_OCTOMAP_LIB_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
# Set library names as absolute paths
# Windows, spec. MSVC requires the .lib suffix for imported libs
IF(WIN32)
  SET(SUPERRAY_OCTOMAP_LIBRARY "${CMAKE_IMPORT_LIBRARY_PREFIX}superray-octomap${CMAKE_IMPORT_LIBRARY_SUFFIX}")
ELSE()
  SET(SUPERRAY_OCTOMAP_LIBRARY "${CMAKE_SHARED_LIBRARY_PREFIX}superray-octomap${CMAKE_SHARED_LIBRARY_SUFFIX}")
ENDIF()

INCLUDE(CMakePackageConfigHelpers)

CONFIGURE_PACKAGE_CONFIG_FILE(
  superray-octomap-config.cmake.in
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/superray-octomap/superray-octomap-config.cmake"
  PATH_VARS SUPERRAY_OCTOMAP_INCLUDE_DIRS SUPERRAY_OCTOMAP_LIB_DIR
  INSTALL_DESTINATION "${CMAKE_INSTALL_FULL_DATADIR}/superray-octomap"
)

# Create a superray-octomap-config.cmake file for the use from the install tree and install it
SET(SUPERRAY_OCTOMAP_INCLUDE_DIRS "${CMAKE_INSTALL_FULL_INCLUDEDIR}/superray")
SET(SUPERRAY_OCTOMAP_LIB_DIR "${CMAKE_INSTALL_FULL_LIBDIR}")

CONFIGURE_PACKAGE_CONFIG_FILE(
  superray-octomap-config.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/superray-octomap-config.cmake"
  PATH_VARS SUPERRAY_OCTOMAP_INCLUDE_DIRS SUPERRAY_OCTOMAP_LIB_DIR
  INSTALL_DESTINATION "${CMAKE_INSTALL_FULL_DATADIR}/superray-octomap"
)

INSTALL(FILES 
  "${PROJECT_BINARY_DIR}/InstallFiles/superray-octomap-config.cmake"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/superray-octomap"
)

# Finished:
MESSAGE (STATUS "\n")
MESSAGE (STATUS "OctoMap library: ${OCTOMAP_VERSION}")
MESSAGE (STATUS "Compile superray-octomap using: make")
MESSAGE (STATUS "Install superray-octomap using: make install")
MESSAGE (STATUS "    (be sure to set the correct CMAKE_INSTALL_PREFIX before)")