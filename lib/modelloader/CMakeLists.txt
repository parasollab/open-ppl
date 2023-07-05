cmake_minimum_required(VERSION 3.24)
#set(CMAKE_CXX_STANDARD 17)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

#get_filename_component(MATHTOOL_PATH ../../include/mathtool ABSOLUTE)

find_package(CGAL CONFIG REQUIRED)

include_directories(${CGAL_INCLUDE_DIRS})

add_library(modelloader STATIC IModel.cpp ModelFactory.cpp MovieBYULoader.cpp ObjLoader.cpp BVHDataLoader.cpp)

target_include_directories(modelloader
PUBLIC
$<INSTALL_INTERFACE:include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_include_directories(modelloader
PUBLIC
$<INSTALL_INTERFACE:include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include/mathtool>
)

target_link_libraries(modelloader PUBLIC CGAL::CGAL)

install(TARGETS modelloader
        EXPORT ModelLoaderTargets
        ARCHIVE DESTINATION  "${CMAKE_INSTALL_LIBDIR}"
        LIBRARY DESTINATION  "${CMAKE_INSTALL_LIBDIR}"

)

install(EXPORT ModelLoaderTargets
        FILE ModelLoaderTargets.cmake
        NAMESPACE MODELLOADER::
        DESTINATION lib/cmake/modelloader
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DESTINATION "include/modelloader"
        FILES_MATCHING PATTERN "*.h"
)

install(FILES
${CMAKE_CURRENT_SOURCE_DIR}/cmake/modelloaderConfig.cmake
DESTINATION "lib/cmake/modelloader" )