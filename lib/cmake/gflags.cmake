set(gflags_DIR ${CMAKE_SOURCE_DIR}/lib/gflags/lib/cmake/gflags)
find_package(gflags REQUIRED)
list(APPEND LINK_LIBS gflags)