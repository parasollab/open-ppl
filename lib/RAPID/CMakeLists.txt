cmake_minimum_required(VERSION 3.24)

project(RAPID LANGUAGES CXX)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(SRCS ${CMAKE_CURRENT_SOURCE_DIR}/RAPID.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/build.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/collide.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/overlap.cpp)


add_library(${PROJECT_NAME} ${SRCS})

target_include_directories(${PROJECT_NAME}
PUBLIC
$<INSTALL_INTERFACE:include/RAPID>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

install(TARGETS RAPID
        EXPORT RapidTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(EXPORT RapidTargets
        FILE RapidTargets.cmake
        NAMESPACE RAPID::
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/rapid-config.cmake 
        DESTINATION share/${PROJECT_NAME} )
