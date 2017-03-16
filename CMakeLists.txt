cmake_minimum_required(VERSION 2.8)
project(icpSparse)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#Adding Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

#Adding Nanoflann
include_directories(${CMAKE_CURRENT_LIST_DIR}/ext/NanoFlann)

#Adding projects headers
include_directories(${CMAKE_CURRENT_LIST_DIR}/lib/ObjLoader)
include_directories(${CMAKE_CURRENT_LIST_DIR}/lib/IcpOptimizer)

#Adding projects libraries
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/ObjLoader)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/IcpOptimizer)

add_executable(icpSparse main.cpp)
target_link_libraries(icpSparse ObjectLoader)
target_link_libraries(icpSparse IcpOptimizer)

