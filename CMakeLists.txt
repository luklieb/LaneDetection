# How to use CMake on Linux with GNU Makefile backend:
#   - Use a separate build folder for intermediate files
#       mkdir build && cd build
#       cmake ..
#       make
#   - you can configure the project for Debug or Release mode by running a cmake GUI in the build folder:
#       cd build
#     then either
#       ccmake . 
#     or 
#       cmake-gui .
#
# There are more generators available, for example for Eclipse, Visual Studio, Ninja, ....
# You can list all available generators with "cmake -G"
#

# add this line to prevent CMake warning: it specifies the minimum cmake version which is required 
# to build this project
#cmake_minimum_required(VERSION 3.1)  

CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( mapra )

INCLUDE_DIRECTORIES( inc )

FILE( GLOB srcs
	"src/*.cpp")

FIND_PACKAGE(OpenCV REQUIRED)
MESSAGE(STATUS "OpenCV library statuss:")
MESSAGE(STATUS "    version: ${OpenCV_VERSION}")
MESSAGE(STATUS "    libraries: ${OpenCV_LIBS}")
MESSAGE(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
#INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

OPTION( ARCHITECTURE_OPTIMAZATION "optimiert" ON )
OPTION( ENABLE_WERROR "werror" ON )
OPTION( NDEBUG "Disables Debug" ON )

IF( NDEBUG )
    ADD_DEFINITIONS( -DNDEBUG )
    MESSAGE(STATUS "Debug disabled")
ELSE()
    MESSAGE(STATUS "Debug enabled")
ENDIF() 

SET( CMAKE_C_COMPILER gcc )
SET( CMAKE_C++_COMPILER gcc )
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wshadow" )

IF( ENABLE_WERROR )
    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror" )
ENDIF()
IF( ARCHITECTURE_OPTIMAZATION )
    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native" )
    MESSAGE(STATUS "Optimizations on")
ENDIF()

ADD_EXECUTABLE( ${CMAKE_PROJECT_NAME} ${srcs} )

TARGET_LINK_LIBRARIES( ${CMAKE_PROJECT_NAME} ${OpenCV_LIBS} )
