set(glfw3_DIR ${CMAKE_SOURCE_DIR}/lib/glfw/lib/cmake/glfw3)
find_package(glfw3 REQUIRED)
list(APPEND LINK_LIBS glfw)