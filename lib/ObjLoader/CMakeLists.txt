file(GLOB_RECURSE oLsourceFiles ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
add_library(ObjectLoader STATIC ${oLsourceFiles})
