CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

SET(PROJECTNAME SLCT)
PROJECT(${PROJECTNAME})
set(CMAKE_CXX_STANDARD 17)
#####################################
# cmake settings
#####################################

#SET(CMAKE_VERBOSE_MAKEFILE ON)
set(PROJECT_ROOT ${CMAKE_SOURCE_DIR}/)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "add a postfix, usually d on windows")  
set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "add a postfix, usually empty on windows")
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)
#SET(CMAKE_AUTOMOC ON)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

#####################################
# compiler flags
#####################################
if(WIN32)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MD")

	set (COMPILE_FLAG
	"${CMAKE_CXX_FLAGS} -DNOMINMAX -DCVX_NO_GUI -DCVX_NO_CODEC /wd4996 /wd4267 /wd4244 /MP")
	# 4996 deprecated crt functions
	# 4267 size to int
	# 4244 float to int

	set(CMAKE_CXX_FLAGS ${COMPILE_FLAG})
	set(CMAKE_C_FLAGS ${COMPILE_FLAG})
else()
	SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
	SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
	#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -c -O3")
	#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MD")
endif()

#####################################
# 3rd libs
#####################################
include(lib/cmake/opengl.cmake)
include(lib/cmake/assimp.cmake)
include(lib/cmake/glog.cmake)
include(lib/cmake/gflags.cmake)
include(lib/cmake/opencv.cmake)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
list(APPEND LINK_LIBS Qt5::Widgets Qt5::OpenGL)

include_directories(src)
#####################################
# here we go
#####################################

add_subdirectory(src/base)
add_subdirectory(src/mbt)
add_subdirectory(src/app)
