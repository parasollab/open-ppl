cmake_minimum_required(VERSION 2.8)

#
# Figure out toolchain files to use in this build
#

set(CMAKE_TOOLCHAIN_PATH "./cmake/Toolchain")

if(NOT DEFINED SYS_TYPE) # If we're on a LLNL system
  set(SYS_TYPE $ENV{SYS_TYPE})
endif()

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  # If the user didn't specify a toolchain, then default the toolchain file
  if(NOT DEFINED TOOLCHAIN)  
    if(SYS_TYPE MATCHES bgqos_0)
      set(CMAKE_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_PATH}/bgqos_0-gcc.cmake")
    elseif(SYS_TYPE MATCHES chaos_5_x86_64_ib)
      set(CMAKE_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_PATH}/chaos_5_x86_64_ib-ic14.cmake")
      include("${CMAKE_TOOLCHAIN_PATH}/chaos_5_x86_64_ib-ic14.cmake")
    elseif(SYS_TYPE MATCHES rain)
      set(CMAKE_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_PATH}/rain.cmake")
      include("${CMAKE_TOOLCHAIN_PATH}/rain.cmake")
    elseif(SYS_TYPE MATCHES optiplex)
      set(CMAKE_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_PATH}/optiplex.cmake")
      include("${CMAKE_TOOLCHAIN_PATH}/optiplex.cmake")
    endif()
  # If the user selected a toolchain (ex. ic14, gcc) then prepend the systype
  else()
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_PATH}/${SYS_TYPE}-${TOOLCHAIN}.cmake")
  endif()
endif()

message(STATUS "Selected toolchain file ${CMAKE_TOOLCHAIN_FILE}")

#
# Invoke the project, choose language, display some debug info
#

project(KRIPKE)

# Re-incluing the toolchain file, which allows us to modify the compiler flags
# since all of the flags are lost after calling project()
include(${CMAKE_TOOLCHAIN_FILE})


set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "0")
set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_SOURCE_IGNORE_FILES "tarball.py;kripke-tarball;.git;.cproject;.project;${CPACK_SOURCE_IGNORE_FILES}")
include(CPack)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

enable_language(CXX)

message(STATUS "Selected Build Type ${CMAKE_BUILD_TYPE}")
message(STATUS "Selected C++ Compiler ${CMAKE_CXX_COMPILER}")
message(STATUS "Selected C++ Flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "Selected BDIV Package Path ${PKG_PATH}")



#
# Libraries to include
#  

# Include macros to aid in BDIV packages
set(BDIV_LIBRARY_PATH "${KRIPKE_SOURCE_DIR}/cmake/Library")
include(cmake/bdiv_pkg.cmake)

# Typically used (turned on by default)
if(SYS_TYPE MATCHES rain) # Leave off until I can get gperftools working
  bdiv_opt_pkg(tcmalloc 0 OFF)
else()
  bdiv_opt_pkg(tcmalloc 0 ON)
endif()

# Optional 
bdiv_opt_pkg(memP 0.0 OFF)
bdiv_opt_pkg(papi 4.2.0 OFF)
bdiv_opt_pkg(perftools 0 OFF)

# BG/Q Specific
bdiv_opt_pkg(BGPM 20030410 OFF)


#
# Other Build Options
# 

# OpenMP build option
include(cmake/OpenMP.cmake)


#
# Process build files for Kripke
#
set(KRIPKE_LIBS kripkelib ${BDIV_LIBS})
set(KRIPKE_LIBS ${KRIPKE_LIBS} ${KRIPKE_LIBS})



add_subdirectory(src)

