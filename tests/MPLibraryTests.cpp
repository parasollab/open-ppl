#include <catch2/catch_test_macros.hpp>

#include "MPLibrary/MPLibrary.h"    //src
#include "Traits/TestTraits.h"

#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Agents/Coordinator.h"
#include "TMPLibrary/Solution/Plan.h"
#include "MPProblem/MPProblem.h"   // no orig, new?
#include "MPProblem/MPTask.h"
#include "MPProblem/GroupTask.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"
#include "MPLibrary/PMPL.h" // new
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"  // no orig
#include "MPLibrary/MPStrategies/ValidationStrategy.h"  // no orig
#include "Utilities/PMPLExceptions.h"  

#include "MPLibrary/MPLibraryTests.h"
#include "TMPLibrary/TMPLibraryTests.h"

#include "MPProblem/MPProblemTests.h"

#include "MPLibrary/DistanceMetrics/DistanceMetricMethodTest.h"

extern std::string test_ufile;

TEST_CASE( "MPLibrary TESTS", "[multi-file:4]" ) {
    // Parse the Problem node into an MPProblem object.
    MPProblemTests* problem = new  MPProblemTests(test_ufile);

    MPLibraryTests<MPTraits<Cfg>>* mpl = new MPLibraryTests<MPTraits<Cfg>>(test_ufile);

    mpl->SetMPProblem(problem);

    // Init mpsolution for stat purposes (and avoiding seg faults)
    mpl->SetMPSolution(new MPSolutionType<MPTraits<Cfg>>(mpl->GetMPProblem()->GetRobotGroups()[0].get()));

    std::cout << "\nMPLibrary Tests: Collision Detection Tests" << std::endl;
    mpl->InitializeCollisionDetectionMethodTests();

    for (auto method : mpl->m_collisionDetectionTests) {
        std::cout << "...Running test for " << method.first << "... " << method.second << std::endl;

        auto test = dynamic_cast<TestBaseObject*>(method.second);
        auto result = test->RunTest();

        std::cout << result.second << std::endl;
        CHECK(result.first == true);
    }

    std::cout << "\nMPLibrary Tests: MPStrategy Tests" << std::endl;
    mpl->InitializeMPStrategyMethodTests();

    for (auto method : mpl->m_mpStrategyTests) {
        std::cout << "...Running test for " << method.first << "... " << method.second << std::endl;

        auto test = dynamic_cast<TestBaseObject*>(method.second);
        auto result = test->RunTest();

        std::cout << result.second << std::endl;
        CHECK(result.first == true);
    }

    std::cout << "\nMPLibrary Tests: Distance Metric Tests" << std::endl;

    auto mtlDistance = *mpl->m_distanceMetricTests;

    for(auto iter = mtlDistance.begin(); iter != mtlDistance.end(); iter++) {
        std::cout << "...Running test for " << iter->first << "..." << std::endl;

        auto test = iter->second.get();
        auto result = test->RunTest();

        std::cout << result.second << std::endl;
        CHECK(result.first == true);
    }

    std::cout << "\nMPLibrary Tests: Validity Checker Tests" << std::endl;

    auto mtlValidity = *mpl->m_validityCheckerTests;

    for(auto iter = mtlValidity.begin(); iter != mtlValidity.end(); iter++) {
        std::cout << "...Running test for " << iter->first << "..." << std::endl;

        auto test = iter->second.get();
        auto result = test->RunTest();

        std::cout << result.second << std::endl;
        CHECK(result.first == true);
    }

    std::cout << "\nMPLibrary Tests: Neighborhood Finder Tests" << std::endl;

    auto mtlNeighborhood = *mpl->m_neighborhoodFinderTests;

    for(auto iter = mtlNeighborhood.begin(); iter != mtlNeighborhood.end(); iter++) {
        std::cout << "...Running test for " << iter->first << "..." << std::endl;

        auto test = iter->second.get();
        auto result = test->RunTest();

        std::cout << result.second << std::endl;
        CHECK(result.first == true);
    }

    std::cout << "\nMPLibrary Tests: Sampler Tests" << std::endl;

    auto mtlSampler = *mpl->m_samplerTests;

    for(auto iter = mtlSampler.begin(); iter != mtlSampler.end(); iter++) {
        std::cout << "...Running test for " << iter->first << "..." << std::endl;

        auto test = iter->second.get();
        auto result = test->RunTest();

        std::cout << result.second << std::endl;
        CHECK(result.first == true);
    }

    std::cout << "\nMPLibrary Tests: Local Planner Tests" << std::endl;

    auto mtlLocalPlanner = *mpl->m_localPlannerTests;

    for(auto iter = mtlLocalPlanner.begin(); iter != mtlLocalPlanner.end(); iter++) {
        std::cout << "...Running test for " << iter->first << "..." << std::endl;

        auto test = iter->second.get();
        auto result = test->RunTest();

        std::cout << result.second << std::endl;
        CHECK(result.first == true);
    }

    std::cout << "\nMPLibrary Tests: Extender Tests" << std::endl;

    auto mtlExtender = *mpl->m_extenderTests;

    for(auto iter = mtlExtender.begin(); iter != mtlExtender.end(); iter++) {
        std::cout << "...Running test for " << iter->first << "..." << std::endl;

        auto test = iter->second.get();
        auto result = test->RunTest();

        std::cout << result.second << std::endl;
        CHECK(result.first == true);
    }

    std::cout << "\nMPLibrary Tests: Path Modifier Tests" << std::endl;

    auto mtlPath = *mpl->m_pathModifierTests;

    for(auto iter = mtlPath.begin(); iter != mtlPath.end(); iter++) {
        std::cout << "...Running test for " << iter->first << "..." << std::endl;

        auto test = iter->second.get();
        auto result = test->RunTest();

        std::cout << result.second << std::endl;
        CHECK(result.first == true);
    }        

    std::cout << "\nMPLibrary Tests: Connector Tests" << std::endl;

    auto mtlConnector = *mpl->m_connectorTests;

    for(auto iter = mtlConnector.begin(); iter != mtlConnector.end(); iter++) {
        std::cout << "...Running test for " << iter->first << "..." << std::endl;

        auto test = iter->second.get();
        auto result = test->RunTest();

        std::cout << result.second << std::endl;
        CHECK(result.first == true);
    }

    std::cout << "\nMPLibrary Tests: Metric Tests" << std::endl;

    auto mtlMetric = *mpl->m_metricTests;

    for(auto iter = mtlMetric.begin(); iter != mtlMetric.end(); iter++) {
        std::cout << "...Running test for " << iter->first << "..." << std::endl;

        auto test = iter->second.get();
        auto result = test->RunTest();

        std::cout << result.second << std::endl;
        CHECK(result.first == true);
    }
    
    std::cout << "\nMPLibrary Tests: Map Evaluator Tests" << std::endl;

    auto mtlMap = *mpl->m_mapEvaluatorTests;

    for(auto iter = mtlMap.begin(); iter != mtlMap.end(); iter++) {
        std::cout << "...Running test for " << iter->first << "..." << std::endl;

        auto test = iter->second.get();
        auto result = test->RunTest();

        std::cout << result.second << std::endl;
        CHECK(result.first == true);
    }

    std::cout << "\nCOMPLETED MPLibrary TESTS\n" << std::endl;

    delete problem;
    delete mpl;
}