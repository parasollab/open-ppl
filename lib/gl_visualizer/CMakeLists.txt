cmake_minimum_required(VERSION 3.24)
set(CMAKE_CXX_STANDARD 17)

project(gl_visualizer)

add_subdirectory(nonstd) #build nonstd using nonstd/CmakeLists.txt
add_subdirectory(glutils) #build nonstd using glutils/CmakeLists.txt
add_subdirectory(sandbox) #build nonstd using sandbox/CmakeLists.txt
