cmake_minimum_required(VERSION 2.8.5)
project(VoxelRecontruction)

set(CMAKE_VERBOSE_MAKEFILE OFF)

message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})
option(WITH_OPENMP      "Build with OpenMP support for multithreading"                  ON)

#set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_BUILD_TYPE MATCHES Debug)
        add_definitions(-O0)
        add_definitions(-g3)
        add_definitions(-DDEBUG)
elseif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
        add_definitions(-O3)
        add_definitions(-g3)
        add_definitions(-DDEBUG)
else(CMAKE_BUILD_TYPE MATCHES Debug)
        set(CMAKE_BUILD_TYPE Release)
        add_definitions(-O3)
        add_definitions(-g0)
        add_definitions(-DNDEBUG)
endif(CMAKE_BUILD_TYPE MATCHES Debug)

add_definitions(-std=c++11)

find_package(OpenCV 2.4 REQUIRED)
find_package(OpenGL 1 REQUIRED)
find_package(GLUT 3 REQUIRED)

if (WITH_OPENMP)
    find_package(OpenMP)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
endif()

include_directories (include)
include_directories (${OPENGL_INCLUDE_DIR})
include_directories (${GLUT_INCLUDE_DIR})
include_directories (${OpenCV_INCLUDE_DIR})

link_directories (
)

#set the default path for built executables to the BUILD_TYPE directory ("Release" or "Debug")
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/${CMAKE_BUILD_TYPE})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/${CMAKE_BUILD_TYPE})

#############################################

add_executable (
	VoxelRecontruction

	#$ find src .|grep -v "\.svn"|grep -v "\./"|grep cpp|sort
	##########
	src/controllers/arcball.cpp
	src/controllers/Camera.cpp
	src/controllers/Glut.cpp
	src/controllers/Reconstructor.cpp
	src/controllers/Scene3DRenderer.cpp
	src/utilities/General.cpp
	src/VoxelReconstruction.cpp
	src/main.cpp
)

#############################################

target_link_libraries (VoxelRecontruction ${OPENGL_LIBRARIES})
target_link_libraries (VoxelRecontruction ${GLUT_LIBRARIES})
target_link_libraries (VoxelRecontruction ${OpenCV_LIBS})
target_link_libraries (VoxelRecontruction)
if(WITH_OPENMP)
	target_link_libraries (VoxelRecontruction gomp)
endif(WITH_OPENMP)

message(${PROJECT_NAME} " Configuration Summary")
message("Build with threading (OpenMP): ${WITH_OPENMP}")
