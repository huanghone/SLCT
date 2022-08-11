set(freeglut_PREFIX "${CMAKE_SOURCE_DIR}/freeglut")
ExternalProject_Add( freeglut
  PREFIX "${freeglut_PREFIX}"
  #GIT_REPOSITORY https://github.com/FreeGLUTProject/freeglut.git
  GIT_REPOSITORY https://gitee.com/huanghone/freeglut.git
  GIT_TAG v3.2.2
  CMAKE_ARGS 
    -DCMAKE_INSTALL_PREFIX=${freeglut_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
    -DFREEGLUT_BUILD_DEMOS=OFF
    -DFREEGLUT_BUILD_STATIC_LIBS=OFF
    -DINSTALL_PDB=OFF
)