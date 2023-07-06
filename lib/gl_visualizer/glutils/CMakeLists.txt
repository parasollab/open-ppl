cmake_minimum_required(VERSION 3.24)
set(CMAKE_CXX_STANDARD 17)


include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

get_filename_component(NONSTD_PATH ../nonstd ABSOLUTE)
include_directories(${NONSTD_PATH})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

option(BUILD_GLUTILS_TESTS "Build and Run Tests" OFF)
option(BUILD_GLUTILS_DOCS "Build Documentation" OFF)

# Build Documentation
if(BUILD_GLUTILS_DOCS)
    find_package(Doxygen REQUIRED dot OPTIONAL_COMPONENTS mscgen dia)

    set(DOXYGEN_GENERATE_HTML YES)
    set(DOXYGEN_GENERATE_MAN NO)

    # configure doxygen configuration file
    set(doxyfile_in docs/Doxygen/Doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    # add custom target for doc generation
    add_custom_target(doc
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)

    doxygen_add_docs(docs ${CMAKE_CURRENT_SOURCE_DIR}/include/ ${CMAKE_CURRENT_SOURCE_DIR}/INSTALL.md  ${CMAKE_CURRENT_SOURCE_DIR}/CHANGELOG.md
            ALL
            COMMENT "Generate documentation with Doxygen")

    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)

    doxygen_add_docs(doxygen ${CMAKE_CURRENT_SOURCE_DIR} COMMENT "Generate man pages")
endif()

if(BUILD_GLUTILS_TESTS)
    enable_testing()
    find_package(Catch2 CONFIG REQUIRED)
endif()


file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/glutils/*.cpp)

add_library(glutils ${SRCS})

target_link_libraries(glutils
        PRIVATE nonstd)

target_include_directories(glutils 
        PUBLIC 
        $<INSTALL_INTERFACE:include/glutils> 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DESTINATION "include"
        FILES_MATCHING PATTERN "*.h"
)
