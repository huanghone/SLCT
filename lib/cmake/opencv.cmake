if(WIN32)
	set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/lib/opencv)
else()
	set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/lib/opencv/share/OpenCV)
endif()
find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
list(APPEND LINK_LIBS ${OpenCV_LIBS})