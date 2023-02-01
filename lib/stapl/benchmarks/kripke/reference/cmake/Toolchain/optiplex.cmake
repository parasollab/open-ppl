
set(CMAKE_C_COMPILER /usr/lib64/mpich/bin/mpicc)

set(CMAKE_CXX_COMPILER /usr/lib64/mpich/bin/mpic++)

set(CMAKE_LINKER /usr/lib64/mpich/bin/mpic++)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -finline-functions -finline-limit=20000")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -finline-functions -finline-limit=20000")

set(PKG_PATH "/usr")

