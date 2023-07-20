cmake_minimum_required(VERSION 3.24)
set(CMAKE_CXX_STANDARD 17)

project(gl_visualizer)

add_subdirectory(nonstd)
add_subdirectory(glutils)
add_subdirectory(sandbox)

install(TARGETS nonstd glutils sandbox
        EXPORT GLVisTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include
)

install(EXPORT GLVisTargets
        FILE glvisTargets.cmake
        NAMESPACE GLVIS::
        DESTINATION lib/cmake/${PROJECT_NAME}
)

install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gl_visualiserConfig.cmake
        DESTINATION share/${PROJECT_NAME} )
