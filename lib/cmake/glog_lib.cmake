set(glog_PREFIX "${CMAKE_SOURCE_DIR}/glog")
ExternalProject_Add( glog
  GIT_REPOSITORY https://gitee.com/huanghone/glog.git
  GIT_TAG v0.6.0
  INSTALL_DIR ${glog_PREFIX}
  CMAKE_ARGS 
    -DCMAKE_INSTALL_PREFIX=${glog_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_SHARED_LIBS=OFF
    -DBUILD_TESTING=OFF
    -DWITH_GFLAGS=OFF
)