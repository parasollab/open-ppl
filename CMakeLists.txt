cmake_minimum_required(VERSION 3.24)
set(CMAKE_CXX_STANDARD 17)
#add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

project(pmpl VERSION 1.0.0)

set(ROBOT_DEF "PMPCfg" CACHE STRING "Robot Type")
add_definitions(-D${ROBOT_DEF})

option(BUILD_ICREATE "Build Support for iCreate Hardware" OFF)
option(BUILD_ARUCO "Build support for aruco detector hardware" OFF)
option(BUILD_MATLAB "Build with MatLab support" OFF)

# Compile the parallel version of PMPL? This is very old and has not been
# maintained during the simulator refactoring (too many things to manage at
# once). At this point it likely needs to be rebuilt entirely using the latest
# work from Matthew Bulluck's parallel branch.
option(BUILD_PARALLEL "Build parallel version of PMPL" OFF)

option(BUILD_TESTS "Build and Run Tests" OFF)
option(BUILD_DOCS "Build Documentation" OFF)

add_compile_definitions(VIZMO_MAP)
add_compile_definitions(CGAL_NO_ASSERTIONS)
add_compile_definitions(CGAL_NO_PRECONDITIONS)
add_compile_definitions(CGAL_NO_POSTCONDITIONS)
add_compile_definitions(CGAL_NO_WARNINGS)

# used by CMake to scan the header and source files at build time and invoke moc accordingly.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add Packages (needs to be installed)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Qt6 CONFIG REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)

set(CMAKE_PREFIX_PATH ${QT_ROOT})

# support for pthreads (if needed in future)
#set(THREADS_PREFER_PTHREAD_FLAG ON)
#find_package(Threads REQUIRED)
#target_link_libraries(my_app PRIVATE Threads::Threads)

# build libraries
add_subdirectory(lib/tetgen)
add_subdirectory(lib/PQP)
add_subdirectory(lib/modelloader)
add_subdirectory(lib/RAPID)

add_subdirectory(lib/gl_visualizer/nonstd)
add_subdirectory(lib/gl_visualizer/glutils)
add_subdirectory(lib/gl_visualizer/sandbox)

# Resolve local packages
set(STAPL_PATH lib/stapl)
find_library(STAPL ${STAPL_PATH})

# Find Packages (resolved by package manager)
find_package(Boost REQUIRED)
find_package(CGAL CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(tinyxml CONFIG REQUIRED)
find_package(Bullet CONFIG REQUIRED)
find_package(dlib CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(pqp)

add_subdirectory(src)
include_directories(src)

# TODO Remove, Added Build Parallel
include_directories(lib/stapl)
include_directories(lib/stapl/stapl)

include_directories(${Qt6Widgets_INCLUDE_DIRS})
include_directories(${Qt6OpenGLWidgets_INCLUDE_DIRS})

# use target_include_directories() if needed for specific targets only
include_directories(include/mathtool)

include_directories(include/LKH/SRC/INCLUDE)

include_directories(lib/gl_visualizer/sandbox)
include_directories(lib/gl_visualizer/glutils)
include_directories(lib/gl_visualizer/nonstd)

include_directories(lib/tetgen)
include_directories(lib/modelloader)
include_directories(lib/RAPID)

# Build Documentation
if(BUILD_DOCS)
    find_package(Doxygen REQUIRED dot OPTIONAL_COMPONENTS mscgen dia)
    add_subdirectory(docs)
endif()

# Build Catch2 Tests
if(BUILD_TESTS)
    enable_testing()
    find_package(Catch2 CONFIG REQUIRED)
    add_subdirectory(tests)
endif()

# Recursively add sources (Not Used ATM)
file(GLOB_RECURSE SRCS src/*.cpp src/*.h)

# include visual studio required file for 'or' and 'and' keywords
if(MSVC)
    add_definitions(/FI"iso646.h")
endif()

# define target name variables
set(PPL_EXECUTABLE ppl_executable)
set(PPL_LIBRARY ppl_library)
set(PMPL_EXECUTABLE pmpl_executable)
set(PMPL_LIBRARY pmpl_library)
# TODO add last target (Simulator?)

# build [base object library]
add_library(objlib OBJECT ${PMPL_SRCS})
set_target_properties(objlib PROPERTIES VERSION ${PROJECT_VERSION})
target_link_libraries(objlib PRIVATE unofficial-tinyxml::unofficial-tinyxml LinearMath Bullet3Common
        BulletDynamics BulletSoftBody BulletCollision BulletInverseDynamics)
target_link_directories(objlib PRIVATE ${BULLET_LIBRARY_DIRS})

# build [pmpl library]
add_library(${PMPL_LIBRARY} ${HARDWARE_SRCS} ${TASK_SRCS} $<TARGET_OBJECTS:objlib>)

# [pmpl library] - Link external packages
target_link_libraries(${PMPL_LIBRARY} PRIVATE LinearMath Bullet3Common BulletDynamics
        BulletSoftBody BulletCollision BulletInverseDynamics CGAL::CGAL dlib::dlib
        Eigen3::Eigen nlohmann_json::nlohmann_json Qt6::Core Qt6::Gui Qt6::OpenGL Qt6::Widgets
        Qt6::OpenGLWidgets)

target_link_libraries(${PMPL_LIBRARY} PRIVATE ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

# [pmpl library] - Link internal built packages
target_link_libraries(${PMPL_LIBRARY} PRIVATE glutils nonstd sandbox RAPID tetgen pqp modelloader)

# [pmpl library] - Link conditional packages
if(BUILD_TESTS)
    target_link_libraries(${PMPL_LIBRARY} PRIVATE Catch2::Catch2 Catch2::Catch2WithMain)
endif()

# [pmpl library] - needed due to issue with bullet library
target_link_directories(${PMPL_LIBRARY} PRIVATE ${BULLET_LIBRARY_DIRS})

set_target_properties(${PMPL_LIBRARY} PROPERTIES OUTPUT_NAME pmpl)

# build [pmpl executable]
add_executable(${PMPL_EXECUTABLE} src/main.cpp)
target_link_libraries(${PMPL_EXECUTABLE} ${PMPLLIB})
target_link_libraries(${PMPL_EXECUTABLE} unofficial-tinyxml::unofficial-tinyxml LinearMath Bullet3Common BulletDynamics
        BulletSoftBody BulletCollision BulletInverseDynamics)
target_link_directories(${PMPL_EXECUTABLE} PRIVATE ${BULLET_LIBRARY_DIRS})
set_target_properties(${PMPL_EXECUTABLE} PROPERTIES OUTPUT_NAME pmpl)

# package pmpl library for distribution
install(TARGETS ${PMPL_LIBRARY}
            LIBRARY DESTINATION lib
            PUBLIC_HEADER DESTINATION include)