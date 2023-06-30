cmake_minimum_required(VERSION 3.24)
set(CMAKE_CXX_STANDARD 17)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# used by CMake to scan the header and source files at build time and invoke moc accordingly.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)  # https://github.com/CGAL/cgal/issues/5899

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Qt6 CONFIG REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)

#get_filename_component(NONSTD_PATH ../nonstd ABSOLUTE)
#get_filename_component(GLUTILS_PATH ../glutils ABSOLUTE)
#include_directories(${NONSTD_PATH})
#include_directories(${GLUTILS_PATH})
include_directories(${Qt6Widgets_INCLUDE_DIRS})
include_directories(${Qt6OpenGLWidgets_INCLUDE_DIRS})

set(CMAKE_PREFIX_PATH ${QT_ROOT})

file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/sandbox/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/sandbox/gui/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/sandbox/gui/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/sandbox/*.h)

add_library(sandbox ${SRCS})

target_include_directories(sandbox
        PUBLIC 
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sandbox>
)

target_link_libraries(sandbox PRIVATE Qt6::Core Qt6::Gui Qt6::OpenGL Qt6::Widgets Qt6::OpenGLWidgets)
target_link_libraries(sandbox PUBLIC glutils)
target_link_libraries(sandbox PUBLIC nonstd)
target_link_libraries(sandbox PRIVATE ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})


install(TARGETS sandbox
        EXPORT SandboxTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/sandbox
        DESTINATION include/sandbox
        FILES_MATCHING PATTERN "*.h"
)

 install(EXPORT SandboxTargets
         FILE sandboxTargets.cmake
         NAMESPACE SANDBOX::
         DESTINATION lib/cmake/sandbox
 )

 install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sandboxConfig.cmake
        DESTINATION lib/cmake/sandbox )