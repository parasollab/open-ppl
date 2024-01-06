cmake_minimum_required(VERSION 3.24)
project(docs)

set(DOXYGEN_GENERATE_HTML YES)
set(DOXYGEN_GENERATE_MAN NO)

# configure doxygen configuration file
set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxygen/Doxyfile.in)
set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxygen/output.doxygen)

# request to configure the file
configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

# add custom target for doc generation
add_custom_target(generate_doxygen_docs ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)