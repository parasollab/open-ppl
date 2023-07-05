cmake_minimum_required(VERSION 3.1)
project(tetgen)

option(BUILD_EXECUTABLE "Build tetgen executable" OFF)
option(BUILD_LIBRARY    "Build libtetgen library" ON)

if(BUILD_LIBRARY)
  add_library(tetgen STATIC tetgen.cxx predicates.cxx)
  target_compile_definitions(tetgen PRIVATE -DTETLIBRARY)
  target_include_directories(tetgen
    PUBLIC 
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  )
  # Generate position independent code
  set_target_properties(tetgen PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()


if(BUILD_EXECUTABLE)
  add_executable(tetgen_exec tetgen.cxx predicates.cxx)
  set_target_properties(tetgen_exec PROPERTIES OUTPUT_NAME tetgen)
endif()


install(TARGETS tetgen
        EXPORT TetgenTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

install(EXPORT TetgenTargets
        FILE TetgenTargets.cmake
        NAMESPACE Tetgen::
        DESTINATION lib/cmake/tetgen
)

install(FILES 
tetgen.h 
DESTINATION "include")

install(FILES
${CMAKE_CURRENT_SOURCE_DIR}/cmake/TetgenConfig.cmake
DESTINATION "lib/cmake/tetgen" )