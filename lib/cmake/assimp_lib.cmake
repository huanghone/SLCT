set(assimp_PREFIX "${CMAKE_SOURCE_DIR}/assimp")
ExternalProject_Add( assimp
  PREFIX "${assimp_PREFIX}"
  GIT_REPOSITORY https://gitee.com/huanghone/assimp.git
  GIT_TAG v5.2.4
  #SOURCE_DIR ${CMAKE_SOURCE_DIR}/assimp-3.3.1
  #URL "D:/code/nonlocal-tracker/lib/assimp/src/assimp-3.3.1.zip"
  #DOWNLOAD_NAME assimp-3.3.1.zip
  CMAKE_ARGS 
    -DCMAKE_INSTALL_PREFIX=${assimp_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
    -DINJECT_DEBUG_POSTFIX=OFF
    -DASSIMP_BUILD_TESTS=OFF
    -DASSIMP_BUILD_ASSIMP_TOOLS=OFF
    -DASSIMP_BUILD_ZLIB=OFF
)