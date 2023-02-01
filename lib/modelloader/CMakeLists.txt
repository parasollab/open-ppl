cmake_minimum_required(VERSION 3.17)
#set(CMAKE_CXX_STANDARD 17)

project(modelloader)

get_filename_component(MATHTOOL_PATH ../../include/mathtool ABSOLUTE)

find_package(CGAL CONFIG REQUIRED)

include_directories(${MATHTOOL_PATH})
include_directories(${CGAL_INCLUDE_DIRS})

add_library(${PROJECT_NAME} IModel.cpp ModelFactory.cpp MovieBYULoader.cpp ObjLoader.cpp BVHDataLoader.cpp)

link_libraries(${PROJECT_NAME} PRIVATE ${CGAL_LIBRARIES})