CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
PROJECT( conversion )

find_package(octomap REQUIRED HINTS ${CMAKE_SOURCE_DIR}/lib/cmake/octomap)
find_package(quadmap REQUIRED HINTS ${CMAKE_SOURCE_DIR}/lib/cmake/quadmap)

SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )

INCLUDE_DIRECTORIES(
    include
    ${CMAKE_SOURCE_DIR}/octomap/include
    ${CMAKE_SOURCE_DIR}/quadmap/include
)

ADD_EXECUTABLE(scan_graph_projection
    src/scan_graph_projection.cpp
)
TARGET_LINK_LIBRARIES(scan_graph_projection
    ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/liboctomap.so
    ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/liboctomath.so
    ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libquadmap.so
    ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libquadmath.so
)