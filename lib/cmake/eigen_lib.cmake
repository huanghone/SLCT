set(eigen_PREFIX "${CMAKE_SOURCE_DIR}/eigen")
ExternalProject_Add( eigen
  GIT_REPOSITORY https://gitee.com/lkhlll/eigen.git
  GIT_TAG 3.3.8
  CMAKE_ARGS 
    -DCMAKE_INSTALL_PREFIX=${eigen_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_TESTING=OFF
)