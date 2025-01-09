CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT( superray-distribution )

set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # enables -fPIC in applicable compilers (required to avoid link errors in some cases)

ADD_SUBDIRECTORY( octomap )
ADD_SUBDIRECTORY( quadmap )
ADD_SUBDIRECTORY( gridmap3D )
ADD_SUBDIRECTORY( gridmap2D )

ADD_SUBDIRECTORY( conversion )