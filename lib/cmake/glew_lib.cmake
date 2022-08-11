set(glew_PREFIX "${CMAKE_SOURCE_DIR}/glew")
ExternalProject_Add( glew
  GIT_REPOSITORY https://github.com/Perlmint/glew-cmake.git
  GIT_TAG glew-cmake-2.2.0
  CMAKE_ARGS 
    -DCMAKE_INSTALL_PREFIX=${glew_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
    -DONLY_LIBS=ON
    -Dglew-cmake_BUILD_SHARED=ON
    -Dglew-cmake_BUILD_STATIC=OFF
)