
set(tcmalloc_PREFIX "${PKG_PATH}/gperftools-2.4-gcc-4.8.4")
set(tcmalloc_DEFAULT_INCLUDE_DIRS "${tcmalloc_PREFIX}/include")
set(tcmalloc_DEFAULT_LIB_DIR "${tcmalloc_PREFIX}/lib")
set(tcmalloc_DEFAULT_LIBS "tcmalloc_minimal")

set(tcmalloc_DEFAULT_DEFINITIONS "-DKRIPKE_USE_TCMALLOC")


