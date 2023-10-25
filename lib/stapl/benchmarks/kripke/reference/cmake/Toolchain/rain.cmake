
set(CMAKE_C_COMPILER /opt/cray/xt-asyncpe/5.24/bin/cc)

set(CMAKE_CXX_COMPILER /opt/cray/xt-asyncpe/5.24/bin/CC)

set(CMAKE_LINKER /opt/cray/xt-asyncpe/5.24/bin/CC)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -march=opteron -finline-functions -finline-limit=20000")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=opteron -finline-functions -finline-limit=20000")

set(PKG_PATH "/mnt/lustre/lus0/crriedel/tools")

