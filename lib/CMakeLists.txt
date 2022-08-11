cmake_minimum_required(VERSION 3.12)
project(libs)

include(ExternalProject)

include(cmake/assimp_lib.cmake)
include(cmake/glog_lib.cmake)
include(cmake/gflags_lib.cmake)
include(cmake/opencv_lib.cmake)
