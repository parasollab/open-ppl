include(CMakeFindDependencyMacro)
find_dependency(Boost REQUIRED)
find_dependency(CGAL REQUIRED)
find_dependency(Eigen3 REQUIRED)
find_dependency(tinyxml2 REQUIRED)
find_dependency(Bullet REQUIRED)
find_dependency(nlohmann_json CONFIG REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/PPLTargets.cmake")
