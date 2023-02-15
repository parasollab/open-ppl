cmake_minimum_required(VERSION 3.15)

set(TEST_SRCS
        tests_main.cpp
        Testing/MPProblem/MPProblemTests.cpp
        Testing/TMPLibrary/TaskAllocators/TaskAllocatorMethodTest.cpp
        Testing/TMPLibrary/TaskDecomposers/TaskDecomposerMethodTest.cpp
        Testing/TMPLibrary/TaskEvaluators/SimpleMotionEvaluatorTest.cpp
        Testing/TMPLibrary/TaskEvaluators/TaskEvaluatorMethodTest.cpp
        Testing/TMPLibrary/TMPLibraryTests.cpp
        Testing/TestBaseObject.cpp

        PARENT_SCOPE
        )



