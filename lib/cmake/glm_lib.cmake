set(glm_PREFIX "${CMAKE_SOURCE_DIR}/glm")
ExternalProject_Add( glm
  GIT_REPOSITORY https://gitee.com/huanghone/glm.git
  GIT_TAG 0.9.3.4
  CMAKE_ARGS 
    -DCMAKE_INSTALL_PREFIX=${glm_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_SHARED_LIBS=ON
    -DBUILD_STATIC_LIBS=ON
    -DGLM_TEST_ENABLE=ON
)