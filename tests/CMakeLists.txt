cmake_minimum_required(VERSION 3.24)
project(tests)

set(TEST_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/MPProblemTests.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TaskAllocators/TaskAllocatorMethodTest.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TaskDecomposers/TaskDecomposerMethodTest.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TaskEvaluators/SimpleMotionEvaluatorTest.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TaskEvaluators/TaskEvaluatorMethodTest.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TMPLibraryTests.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TestBaseObject.cpp

        PARENT_SCOPE
        )

