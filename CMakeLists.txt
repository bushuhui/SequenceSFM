cmake_minimum_required(VERSION 2.8)


project(SequenceSFM)
include(cmake/PICMake.cmake)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message("CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DV3DLIB_ENABLE_SUITESPARSE")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--retain-symbols-file=${CMAKE_SOURCE_DIR}/cmake/static_export.sym")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--version-script=${CMAKE_CURRENT_LIST_DIR}/cmake/shared_export.map")
endif()


find_package(Qt REQUIRED)

INCLUDE_DIRECTORIES(src
    Thirdparty
    Thirdparty/eigen3
    Thirdparty/OpenSURF/src
    Thirdparty/SSBA-3.0
    Thirdparty/pba/pba)

pi_add_target(SequenceSFM BIN
    src
    Thirdparty/OpenSURF/src
    Thirdparty/QGLViewer
    Thirdparty/SSBA-3.0/Geometry
    Thirdparty/SSBA-3.0/Math
    REQUIRED Qt OpenCV OpenGL SuiteSparse System)
pi_report_targets()

