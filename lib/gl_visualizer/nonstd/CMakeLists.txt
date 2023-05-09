cmake_minimum_required(VERSION 3.24)
set(CMAKE_CXX_STANDARD 17)

project(nonstd)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

option(BUILD_NONSTD_TESTS "Build and Run Tests" OFF)
option(BUILD_NONSTD_DOCS "Build Documentation" OFF)

# Build Documentation
if(BUILD_NONSTD_DOCS)
    find_package(Doxygen REQUIRED dot OPTIONAL_COMPONENTS mscgen dia)

    set(DOXYGEN_GENERATE_HTML YES)
    set(DOXYGEN_GENERATE_MAN NO)

    # configure doxygen configuration file
    set(doxyfile_in docs/Doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    # add custom target for doc generation
    add_custom_target(doc
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)

    doxygen_add_docs(docs ${PROJECT_SOURCE_DIR}/include/ ${PROJECT_SOURCE_DIR}/INSTALL.md  ${PROJECT_SOURCE_DIR}/CHANGELOG.md
            ALL
            COMMENT "Generate documentation with Doxygen")

    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)

    doxygen_add_docs(doxygen ${PROJECT_SOURCE_DIR} COMMENT "Generate man pages")
endif()

if(BUILD_NONSTD_TESTS)
    enable_testing()
    find_package(Catch2 CONFIG REQUIRED)
endif()

# include visual studio required file for 'or' and 'and' keywords
if(MSVC)
    add_definitions(/FI"iso646.h")
endif()

file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/*.cpp
        ${PROJECT_SOURCE_DIR}/nonstd/*.cpp
        ${PROJECT_SOURCE_DIR}/*.h
        ${PROJECT_SOURCE_DIR}/*.h)

add_library(${PROJECT_NAME} ${SRCS})

install(TARGETS nonstd
        EXPORT NonStdTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(EXPORT NonStdTargets
        FILE NonStdTargets.cmake
        NAMESPACE NONSTD::
        DESTINATION lib/cmake/nonstd
)