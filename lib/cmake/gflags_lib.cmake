set(gflags_PREFIX "${CMAKE_SOURCE_DIR}/gflags")
ExternalProject_Add( gflags
  PREFIX "${gflags_PREFIX}"
  GIT_REPOSITORY https://gitee.com/huanghone/gflags.git
  GIT_TAG v2.2.2
  INSTALL_DIR ${gflags_PREFIX}
  CMAKE_ARGS 
    -DCMAKE_INSTALL_PREFIX=${gflags_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
    -DREGISTER_INSTALL_PREFIX=OFF
)