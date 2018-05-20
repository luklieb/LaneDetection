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
PROJECT( lanedet )

INCLUDE_DIRECTORIES( inc )

FILE( GLOB srcs
	"src/*.cpp")

SET( OpenCV_DIR "/home/pi/libs/opencv-3.1.1-optimized-pi2/share/OpenCV" )
FIND_PACKAGE(OpenCV REQUIRED)
MESSAGE(STATUS "OpenCV library statuss:")
MESSAGE(STATUS "    version: ${OpenCV_VERSION}")
MESSAGE(STATUS "    libraries: ${OpenCV_LIBS}")
MESSAGE(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
#INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})


find_package(OpenMP)
if (OPENMP_FOUND)
	MESSAGE(STATUS "OPENCV found")
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


OPTION( ARCHITECTURE_OPTIMAZATION "optimiert" ON )
OPTION( ENABLE_WERROR "werror" ON )
OPTION( NDEBUG "Disables Debug" ON )
OPTION( OPT_REPORT "Report about optimzations" ON )

IF( OPT_REPORT )
	ADD_DEFINITIONS( -fopt-info-missed-optimized=report.opt )
	MESSAGE(STATUS "opt report on")
ELSE()
	MESSAGE(STATUS "opt report off")
ENDIF()

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
    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3  -mcpu=cortex-a7  -mfpu=neon-vfpv4 -ftree-vectorize -mfloat-abi=hard" )
    MESSAGE(STATUS "Optimizations on")
ENDIF()

ADD_EXECUTABLE( ${CMAKE_PROJECT_NAME} ${srcs} )

TARGET_LINK_LIBRARIES( ${CMAKE_PROJECT_NAME} ${OpenCV_LIBS} )
