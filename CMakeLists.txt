cmake_minimum_required(VERSION 2.8)
project(icpSparse)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#Adding reference to the media directory
add_definitions(-DICPSPARSE_MEDIA_DIR="\\"${CMAKE_CURRENT_SOURCE_DIR}/media/\\"")

#Adding current directory for include
include_directories(${CMAKE_CURRENT_LIST_DIR})

#Adding OptionParser
include_directories(${CMAKE_CURRENT_LIST_DIR}/ext/OptionParser)

#Adding Eigen
include_directories(${CMAKE_CURRENT_LIST_DIR}/ext/Eigen)

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

