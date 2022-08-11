set(assimp_DIR ${CMAKE_SOURCE_DIR}/lib/assimp/lib/cmake/assimp-5.2)
find_package(assimp REQUIRED)
list(APPEND LINK_LIBS assimp::assimp)