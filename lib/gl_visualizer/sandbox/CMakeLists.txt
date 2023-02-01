cmake_minimum_required(VERSION 3.24)
set(CMAKE_CXX_STANDARD 17)

project(sandbox)

# used by CMake to scan the header and source files at build time and invoke moc accordingly.
set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTORCC OFF)
set(CMAKE_AUTOUIC OFF)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Qt6 CONFIG REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)

get_filename_component(NONSTD_PATH ../nonstd ABSOLUTE)
get_filename_component(GLUTILS_PATH ../glutils ABSOLUTE)
include_directories(${NONSTD_PATH})
include_directories(${GLUTILS_PATH})

include_directories(${Qt6Widgets_INCLUDE_DIRS})
include_directories(${Qt6OpenGLWidgets_INCLUDE_DIRS})

set(CMAKE_PREFIX_PATH ${QT_ROOT})

file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/sandbox/*.cpp
        ${PROJECT_SOURCE_DIR}/sandbox/gui/*.cpp
        ${PROJECT_SOURCE_DIR}/sandbox/gui/*.h
        ${PROJECT_SOURCE_DIR}/sandbox/*.h)

add_library(${PROJECT_NAME} ${SRCS})

target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::Core Qt6::Gui Qt6::OpenGL Qt6::Widgets Qt6::OpenGLWidgets)
target_link_libraries(${PROJECT_NAME} PRIVATE glutils)
target_link_libraries(${PROJECT_NAME} PRIVATE nonstd)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})