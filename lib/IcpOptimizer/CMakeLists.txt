file(GLOB_RECURSE iOsourceFiles ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
add_library(IcpOptimizer STATIC ${iOsourceFiles})
target_link_libraries(IcpOptimizer ObjectLoader)
