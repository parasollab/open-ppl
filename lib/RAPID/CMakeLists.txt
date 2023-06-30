cmake_minimum_required(VERSION 3.24)

project(RAPID LANGUAGES CXX)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(SRCS ${CMAKE_CURRENT_SOURCE_DIR}/RAPID.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/build.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/collide.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/overlap.cpp)


set(HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/matvec.H
            ${CMAKE_CURRENT_SOURCE_DIR}/moments.H
            ${CMAKE_CURRENT_SOURCE_DIR}/obb.H
             ${CMAKE_CURRENT_SOURCE_DIR}/overlap.H
             ${CMAKE_CURRENT_SOURCE_DIR}/RAPID.H)

add_library(${PROJECT_NAME} ${SRCS} ${HEADERS})

target_include_directories(${PROJECT_NAME}
PUBLIC
$<INSTALL_INTERFACE:include>
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
        DESTINATION lib/cmake/rapid
)

install(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rapid)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/RapidConfig.cmake DESTINATION lib/cmake/rapid )