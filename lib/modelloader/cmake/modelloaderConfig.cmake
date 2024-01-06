include(CMakeFindDependencyMacro)

find_dependency(CGAL REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/ModelLoaderTargets.cmake")