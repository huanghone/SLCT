set(FreeGLUT_DIR ${CMAKE_SOURCE_DIR}/lib/freeglut/lib/cmake/FreeGLUT)
find_package(FreeGLUT REQUIRED)
list(APPEND LINK_LIBS FreeGLUT::freeglut)