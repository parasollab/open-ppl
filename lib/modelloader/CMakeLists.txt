cmake_minimum_required(VERSION 3.24)
#set(CMAKE_CXX_STANDARD 17)

project(modelloader)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

get_filename_component(MATHTOOL_PATH ../../include/mathtool ABSOLUTE)

find_package(CGAL CONFIG REQUIRED)

include_directories(${MATHTOOL_PATH})
include_directories(${CGAL_INCLUDE_DIRS})

add_library(${PROJECT_NAME} STATIC IModel.cpp ModelFactory.cpp MovieBYULoader.cpp ObjLoader.cpp BVHDataLoader.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC ${CGAL_LIBRARIES})

install(TARGETS modelloader
        EXPORT ModelLoaderTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(EXPORT ModelLoaderTargets
        FILE ModelLoaderTargets.cmake
        NAMESPACE MODELLOADER::
        DESTINATION lib/cmake/modelloader
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/modelloader
        FILES_MATCHING PATTERN "*.h"
)