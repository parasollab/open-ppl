
set(CMAKE_C_COMPILER mpixlcxx_r)

set(CMAKE_CXX_COMPILER mpixlcxx_r)

#set(CMAKE_LINKER "mpixlcxx_r")

#set(CMAKE_LINKER "memcheck_link mpixlcxx_r")


#set(CMAKE_C_COMPILER /usr/local/tools/compilers/ibm/mpixlc_r-lompbeta2-fastmpi)

#set(CMAKE_CXX_COMPILER /usr/local/tools/compilers/ibm/mpixlcxx_r-lompbeta2-fastmpi)

#set(CMAKE_LINKER /usr/local/tools/compilers/ibm/mpixlcxx_r-lompbeta2-fastmpi)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0") # -qarch=auto")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0") # -qsimd=auto -qhot=novector -qnostrict -g -qreport -qsource -qlist -qlistfmt=html")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -g") # -Wl,-zmuldefs")

set(PKG_PATH "/usr/gapps/tamu/${SYS_TYPE}")

