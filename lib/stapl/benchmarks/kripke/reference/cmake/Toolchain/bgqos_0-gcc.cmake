
set(CMAKE_C_COMPILER /usr/local/tools/compilers/ibm/mpigcc-4.8.4)

set(CMAKE_CXX_COMPILER /usr/local/tools/compilers/ibm/mpig++-4.8.4)

set(CMAKE_LINKER /usr/local/tools/compilers/ibm/mpig++-4.8.4)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -mcpu=a2 -mtune=a2 -finline-functions -finline-limit=20000 -std=c++11 -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mcpu=a2 -mtune=a2 -finline-functions -finline-limit=20000 -std=c++11 -fopenmp")

set(PKG_PATH "/usr/gapps/tamu/${SYS_TYPE}")

