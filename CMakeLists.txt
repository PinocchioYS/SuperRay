CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
PROJECT( superray-distribution )

ENABLE_TESTING()  # enable CTest environment of subprojects

ADD_SUBDIRECTORY( octomap )
ADD_SUBDIRECTORY( quadmap )
ADD_SUBDIRECTORY( superray )

