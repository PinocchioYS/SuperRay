CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
PROJECT( superray-distribution )

ENABLE_TESTING()  # enable CTest environment of subprojects
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # enables -fPIC in applicable compilers (required to avoid link errors in some cases)

ADD_SUBDIRECTORY( octomap )
ADD_SUBDIRECTORY( quadmap )
ADD_SUBDIRECTORY( gridmap3D )
#ADD_SUBDIRECTORY( gridmap2D )

