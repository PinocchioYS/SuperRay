CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
PROJECT( quadmap )

include(CTest)

# version (e.g. for packaging)
set(QUADMAP_MAJOR_VERSION 1)
set(QUADMAP_MINOR_VERSION 0)
set(QUADMAP_PATCH_VERSION 0)
set(QUADMAP_VERSION ${QUADMAP_MAJOR_VERSION}.${QUADMAP_MINOR_VERSION}.${QUADMAP_PATCH_VERSION})
set(QUADMAP_SOVERSION ${QUADMAP_MAJOR_VERSION}.${QUADMAP_MINOR_VERSION})
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

# QUADMAP_OMP = enable OpenMP parallelization (experimental, defaults to OFF)
SET(QUADMAP_OMP FALSE CACHE BOOL "Enable/disable OpenMP parallelization")
IF(DEFINED ENV{QUADMAP_OMP})
  SET(QUADMAP_OMP $ENV{QUADMAP_OMP})
ENDIF(DEFINED ENV{QUADMAP_OMP})
IF(QUADMAP_OMP)
  FIND_PACKAGE( OpenMP REQUIRED)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF(QUADMAP_OMP)

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

file(GLOB quadmap_HDRS ${PROJECT_SOURCE_DIR}/include/quadmap/*.h ${PROJECT_SOURCE_DIR}/include/quadmap/*.hxx)
install(FILES ${quadmap_HDRS}	DESTINATION include/superray/quadmap)
file(GLOB quadmap_math_HDRS ${PROJECT_SOURCE_DIR}/include/quadmap/math/*.h)
install(FILES ${quadmap_math_HDRS}	DESTINATION include/superray/quadmap/math)
file(GLOB quadmap_superray_HDRS ${PROJECT_SOURCE_DIR}/include/quadmap_superray/*.h)
install(FILES ${quadmap_superray_HDRS}	DESTINATION include/superray/quadmap_superray)
file(GLOB quadmap_cullingregion_HDRS ${PROJECT_SOURCE_DIR}/include/quadmap_cullingregion/*.h)
install(FILES ${quadmap_cullingregion_HDRS}	DESTINATION include/superray/quadmap_cullingregion)

# Install catkin package.xml
install(FILES package.xml DESTINATION share/superray_quadmap)

# uninstall target
#configure_file(
#    "${PROJECT_SOURCE_DIR}/CMakeModules/CMakeUninstall.cmake.in"
#    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
#    IMMEDIATE @ONLY)

#add_custom_target(uninstall
#    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE superray_quadmap)
 
# Create a quadmap-config.cmake file for the use from the build tree
set(QUADMAP_INCLUDE_DIRS "${INCLUDE_DIRS}")
set(QUADMAP_LIB_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
# Set library names as absolute paths
# Windows, spec. MSVC requires the .lib suffix for imported libs
IF(WIN32)
  set(QUADMAP_LIBRARY
    "${CMAKE_IMPORT_LIBRARY_PREFIX}quadmap${CMAKE_IMPORT_LIBRARY_SUFFIX}"
  )
  set(QUADMATH_LIBRARY
    "${CMAKE_IMPORT_LIBRARY_PREFIX}quadmath${CMAKE_IMPORT_LIBRARY_SUFFIX}"
  )
ELSE()
  set(QUADMAP_LIBRARY
    "${CMAKE_SHARED_LIBRARY_PREFIX}quadmap${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )
  set(QUADMATH_LIBRARY
    "${CMAKE_SHARED_LIBRARY_PREFIX}quadmath${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )
ENDIF()

include(CMakePackageConfigHelpers)

CONFIGURE_PACKAGE_CONFIG_FILE( quadmap-config.cmake.in
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/quadmap/quadmap-config.cmake"
  PATH_VARS QUADMAP_INCLUDE_DIRS QUADMAP_LIB_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/quadmap
)

WRITE_BASIC_PACKAGE_VERSION_FILE(
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/quadmap/quadmap-config-version.cmake"
  VERSION ${QUADMAP_VERSION}
  COMPATIBILITY AnyNewerVersion)

# Create a quadmap-config.cmake file for the use from the install tree
# and install it
set(QUADMAP_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include/superray")
set(QUADMAP_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib/superray")

CONFIGURE_PACKAGE_CONFIG_FILE( quadmap-config.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/superray_quadmap-config.cmake"
  PATH_VARS QUADMAP_INCLUDE_DIRS QUADMAP_LIB_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/superray_quadmap)

WRITE_BASIC_PACKAGE_VERSION_FILE(
  ${PROJECT_BINARY_DIR}/InstallFiles/superray_quadmap-config-version.cmake
  VERSION ${QUADMAP_VERSION}
  COMPATIBILITY AnyNewerVersion)

install(FILES
  "${PROJECT_BINARY_DIR}/InstallFiles/superray_quadmap-config.cmake"
  "${PROJECT_BINARY_DIR}/InstallFiles/superray_quadmap-config-version.cmake"
  DESTINATION share/superray_quadmap/)

# Write pkgconfig-file:
include(InstallPkgConfigFile)
install_pkg_config_file(quadmap
    CFLAGS
    LIBS -lquadmap -lquadmath
    REQUIRES
    VERSION ${QUADMAP_VERSION})

# Needs to be last statement:
INCLUDE(CPackSettings)

# Finished:
MESSAGE ("\n")
MESSAGE (STATUS "Compile quadmap using: make")
MESSAGE (STATUS "Install quadmap using: make install")
MESSAGE (STATUS "    (be sure to set the correct CMAKE_INSTALL_PREFIX before)")
