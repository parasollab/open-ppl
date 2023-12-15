cmake_minimum_required(VERSION 3.24)
set(CMAKE_CXX_STANDARD 17)
#add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

project(ppl VERSION 1.1.0)

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
option(CODE_COVERAGE "Enable code coverage reporting" OFF)
option(BUILD_DOCS "Build Documentation" OFF)

add_compile_definitions(VIZMO_MAP)
add_compile_definitions(CGAL_NO_ASSERTIONS)
add_compile_definitions(CGAL_NO_PRECONDITIONS)
add_compile_definitions(CGAL_NO_POSTCONDITIONS)
add_compile_definitions(CGAL_NO_WARNINGS)

# used by CMake to scan the header and source files at build time and invoke moc accordingly.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)  #https://github.com/CGAL/cgal/issues/5899

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


# ------------- add ppl dependencies --------------

# Add Packages (needs to be installed)
#find_package(OpenGL REQUIRED)
#find_package(GLUT REQUIRED)
find_package(Qt6 CONFIG COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets REQUIRED)

# set(CMAKE_PREFIX_PATH ${QT_ROOT})

# support for pthreads (if needed in future)
#set(THREADS_PREFER_PTHREAD_FLAG ON)
#find_package(Threads REQUIRED)
#target_link_libraries(my_app PRIVATE Threads::Threads)

# build libraries
add_subdirectory(lib/tetgen)
add_subdirectory(lib/PQP)
add_subdirectory(lib/modelloader)
add_subdirectory(lib/RAPID)
add_subdirectory(lib/gl_visualizer)


# Resolve local packages
set(STAPL_PATH lib/stapl)
find_library(STAPL ${STAPL_PATH})

# Find Packages (resolved by package manager)
find_package(Boost REQUIRED)
find_package(CGAL CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)
find_package(Bullet CONFIG REQUIRED
        LinearMath Bullet3Common BulletDynamics BulletSoftBody BulletCollision BulletInverseDynamics)
find_package(nlohmann_json CONFIG REQUIRED)

add_subdirectory(src)

# TOOD Parallel Build (inclde libstapl.a)
# include_directories(lib/stapl)
# include_directories(lib/stapl/stapl)

# include_directories(${Qt6Widgets_INCLUDE_DIRS})
# include_directories(${Qt6OpenGLWidgets_INCLUDE_DIRS})

# -------------  build ppl targets  --------------

# Build Documentation
if(BUILD_DOCS)
    find_package(Doxygen REQUIRED dot OPTIONAL_COMPONENTS mscgen dia)
    add_subdirectory(docs)
endif()

# Build Catch2 Tests
if(BUILD_TESTS)
    enable_testing()
    find_package(Catch2 3 REQUIRED)

    if(CODE_COVERAGE)
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            message("Enabling code coverage")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
        else()
            message(WARNING "Unsupported compiler for code coverage. Supported compilers: GCC, Clang")
        endif()
    endif()
endif()

# include visual studio required file for 'or' and 'and' keywords
if(MSVC)
    add_definitions(/FI"iso646.h")
endif()

# define target name variables
set(PPL_EXECUTABLE ppl_executable)
set(PPL_LIBRARY ppl_library)
set(PPL_MP_LIBRARY ppl_mp_library)
set(PPL_MP_EXECUTABLE ppl_mp_executable)
set(SIMULATOR simulator)
set(PPL_XML_EXECUTABLE ppl_xml_executable)

# build [base object library]
add_library(objlib OBJECT ${PMPL_SRCS})
set_target_properties(objlib PROPERTIES VERSION ${PROJECT_VERSION})

target_include_directories(objlib 
        PUBLIC 
        $<INSTALL_INTERFACE:include/mathtool>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mathtool>
)

target_include_directories(objlib 
        PUBLIC 
        $<INSTALL_INTERFACE:include/ppl>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_include_directories(objlib 
        PUBLIC 
        $<INSTALL_INTERFACE:include/LKH/SRC/INCLUDE>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/LKH/SRC/INCLUDE>
)

target_include_directories(objlib 
        PUBLIC 
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/stapl>
)
# Must include this path specifically since the .h files are sometimes reference without canonical prefix
target_include_directories(objlib 
        PUBLIC 
        $<INSTALL_INTERFACE:include/stapl>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/stapl/stapl>
)

target_compile_options(objlib PRIVATE
  -Wno-terminate
)

# bullet order is very specific, do not modify
target_link_libraries(objlib
        PUBLIC ${BULLET_LIBRARIES} tinyxml2::tinyxml2)
target_link_libraries(objlib PUBLIC modelloader RAPID tetgen PQP glutils nonstd sandbox)

# build [ppl library]
add_library(${PPL_LIBRARY} STATIC ${HARDWARE_SRCS} ${TASK_SRCS})

target_compile_options(${PPL_LIBRARY} PRIVATE
  -Wno-terminate
)

# build [ppl_mp library]
add_library(${PPL_MP_LIBRARY} STATIC ${HARDWARE_SRCS})

target_compile_options(${PPL_MP_LIBRARY} PRIVATE
  -Wno-terminate
)

target_include_directories(${PPL_LIBRARY}
        PUBLIC 
        $<INSTALL_INTERFACE:include/ppl>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_include_directories(${PPL_LIBRARY}
        PUBLIC
        $<INSTALL_INTERFACE:include/mathtool>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mathtool>
)
target_include_directories(${PPL_LIBRARY} 
        PUBLIC 
        $<INSTALL_INTERFACE:include/LKH/SRC/INCLUDE>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/LKH/SRC/INCLUDE>
)

target_include_directories(${PPL_MP_LIBRARY} 
        PUBLIC 
        $<INSTALL_INTERFACE:include/ppl>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_include_directories(${PPL_MP_LIBRARY}
        PUBLIC
        $<INSTALL_INTERFACE:include/mathtool>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mathtool>
)
target_include_directories(${PPL_MP_LIBRARY} 
        PUBLIC
        $<INSTALL_INTERFACE:include/LKH/SRC/INCLUDE>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/LKH/SRC/INCLUDE>
)

# [ppl library] - Link external packages
# bullet order is very specific, do not modify
target_link_libraries(${PPL_LIBRARY} PUBLIC objlib ${BULLET_LIBRARIES} CGAL::CGAL Eigen3::Eigen
        nlohmann_json::nlohmann_json Qt6::Core Qt6::Gui Qt6::OpenGL Qt6::Widgets
        Qt6::OpenGLWidgets tinyxml2::tinyxml2)
#target_link_libraries(${PPL_LIBRARY} PUBLIC ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
# [ppl library] - Link internal built packages
target_link_libraries(${PPL_LIBRARY} PUBLIC modelloader RAPID tetgen PQP glutils nonstd sandbox)
set_target_properties(${PPL_LIBRARY} PROPERTIES OUTPUT_NAME ppl)

# [ppl_mp library] - Link external packages
target_link_libraries(${PPL_MP_LIBRARY} PUBLIC objlib ${BULLET_LIBRARIES} CGAL::CGAL Eigen3::Eigen
        nlohmann_json::nlohmann_json Qt6::Core Qt6::Gui Qt6::OpenGL Qt6::Widgets
        Qt6::OpenGLWidgets tinyxml2::tinyxml2)
#target_link_libraries(${PPL_MP_LIBRARY} PUBLIC ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
# [ppl_mp library] - Link internal built packages
target_link_libraries(${PPL_MP_LIBRARY} PUBLIC PQP glutils nonstd sandbox modelloader RAPID tetgen)
set_target_properties(${PPL_MP_LIBRARY} PROPERTIES OUTPUT_NAME ppl_mp)

# build [ppl executable]
add_executable(${PPL_EXECUTABLE} src/tmp_main.cpp)
target_link_libraries(${PPL_EXECUTABLE} PRIVATE ${PPL_LIBRARY} RAPID tetgen)
set_target_properties(${PPL_EXECUTABLE} PROPERTIES OUTPUT_NAME ppl)

# build [ppl_mp executable]
add_executable(${PPL_MP_EXECUTABLE} src/main.cpp)
target_link_libraries(${PPL_MP_EXECUTABLE} PRIVATE ${PPL_MP_LIBRARY})
set_target_properties(${PPL_MP_EXECUTABLE} PROPERTIES OUTPUT_NAME ppl_mp)

# build [sim executable]
add_executable(${SIMULATOR} src/sim_main.cpp)
target_link_libraries(${SIMULATOR} PRIVATE ${PPL_LIBRARY})
set_target_properties(${SIMULATOR} PROPERTIES OUTPUT_NAME sim)

# build [tests]
if(BUILD_TESTS)
    set(TESTS_UNIT unit_tests)
    set(TESTS_STRATEGY strategy_tests)

    include(CTest)
    include(Catch)

    add_executable(${TESTS_UNIT} tests/tests_main.cpp
        tests/TestBaseObject.cpp
        tests/MPProblem/MPProblemTests.cpp
        tests/TMPLibrary/TMPLibraryTests.cpp
        tests/TMPLibrary/TaskAllocators/TaskAllocatorMethodTest.cpp
        tests/TMPLibrary/TaskDecomposers/TaskDecomposerMethodTest.cpp
        tests/TMPLibrary/TaskEvaluators/SimpleMotionEvaluatorTest.cpp
        tests/TMPLibrary/TaskEvaluators/TaskEvaluatorMethodTest.cpp
        tests/GeometryTests.cpp
        tests/BehaviorTests.cpp
        tests/MPLibraryTests.cpp    
    )

    target_link_libraries(${TESTS_UNIT} PRIVATE ${PPL_LIBRARY})
    target_link_libraries(${TESTS_UNIT} PRIVATE Catch2::Catch2)
    set_target_properties(${TESTS_UNIT} PROPERTIES OUTPUT_NAME ppl_unit_tests)
    catch_discover_tests(${TESTS_UNIT})

    add_executable(${TESTS_STRATEGY} tests/tests_strategy_main.cpp
        tests/StrategyTests/StrategyTests.cpp
    )

    target_link_libraries(${TESTS_STRATEGY} PRIVATE ${PPL_LIBRARY})
    target_link_libraries(${TESTS_STRATEGY} PRIVATE Catch2::Catch2)
    set_target_properties(${TESTS_STRATEGY} PROPERTIES OUTPUT_NAME ppl_strategy_tests)
    catch_discover_tests(${TESTS_STRATEGY})
endif()

add_executable(${PPL_XML_EXECUTABLE} src/xml_main.cpp)
target_link_libraries(${PPL_XML_EXECUTABLE} PRIVATE ${PPL_LIBRARY})


# -------------  package ppl library for distribution --------------
message("Generating targets for packaging")
include(GNUInstallDirs)

install(TARGETS ${PPL_LIBRARY} ${PPL_MP_LIBRARY} objlib
        EXPORT PPLTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/
        DESTINATION include/ppl
        FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/stapl
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)

install(EXPORT PPLTargets
        FILE PPLTargets.cmake
        NAMESPACE ppl::
        DESTINATION lib/cmake/ppl)

# Create a ConfigVersion.cmake
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/PPLConfigVersion.cmake"
                                    VERSION ${PPL_VERSION}
                                    COMPATIBILITY AnyNewerVersion)

install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/PPLConfigVersion.cmake
        DESTINATION "lib/cmake/ppl")

install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/PPLConfig.cmake
        DESTINATION "lib/cmake/ppl")