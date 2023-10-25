
set(tcmalloc_PREFIX "${PKG_PATH}")
set(tcmalloc_DEFAULT_INCLUDE_DIRS "${tcmalloc_PREFIX}/include/gperftools")
set(tcmalloc_DEFAULT_LIB_DIR "${tcmalloc_PREFIX}/")
set(tcmalloc_DEFAULT_LIBS "tcmalloc_minimal")

set(tcmalloc_DEFAULT_DEFINITIONS "-DKRIPKE_USE_TCMALLOC")


