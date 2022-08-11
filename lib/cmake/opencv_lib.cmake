set(opencv_PREFIX "${CMAKE_SOURCE_DIR}/opencv")
ExternalProject_Add( opencv
  GIT_REPOSITORY https://gitee.com/mirrors/opencv.git
  GIT_TAG 3.4.18
  #GIT_TAG 4.6.0
  INSTALL_DIR ${opencv_PREFIX}
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${opencv_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_SHARED_LIBS=ON
    -DWITH_IPP=OFF
    -DWITH_OPENGL=ON
    -DBUILD_opencv_js=OFF
    -DBUILD_IPP_IW=OFF
    -DBUILD_JAVA=OFF
    -DBUILD_openv_apps=OFF
    -DBUILD_opencv_dnn=OFF
    -DBUILD_opencv_objdetect=OFF
    -DBUILD_opencv_superres=OFF
    -DBUILD_opencv_flann=ON
    -DBUILD_opencv_stitching=OFF
    -DBUILD_opencv_shape=OFF
    -DBUILD_opencv_ml=OFF
    -DBUILD_opencv_photo=OFF
    -DBUILD_opencv_videostab=OFF
    -DBUILD_opencv_java_bindings_generator=OFF
    -DBUILD_opencv_python_bindings_generator=OFF
    -DBUILD_opencv_python_tests=OFF
    -DBUILD_opencv_js_bindings_generator=OFF
    -DBUILD_opencv_ts=OFF
    -DBUILD_PERF_TEST=OFF
    -DBUILD_TEST=OFF
    -DINSTALL_PDB=OFF
    -DINSTALL_PDB_COMPONENT_EXCLUDE_FROM_ALL=OFF
)