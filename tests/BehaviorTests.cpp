#include <catch2/catch_test_macros.hpp>

#include "Behaviors/Agents/CoordinatorTest.h"
#include "MPProblem/MPProblemTests.h"
#include "Simulator/Simulation.h"   //src

extern std::string test_ufile;

TEST_CASE( "Behaviors TESTS", "[multi-file:3]" ) {
    typedef std::pair<bool,std::string> TestResult;

    XMLNode* coordinatorNode{nullptr};
    CoordinatorTest* coordinatorTest{nullptr};
                
    // Parse the Problem node into an MPProblem object.
    MPProblemTests* problem = new  MPProblemTests(test_ufile);

    // Create the simulation object needed for coordinator initialization
    Simulation::Create(problem);

    // Get the coordinator node from the XML file
    XMLNode input(test_ufile, "Problem");

    for(auto& child : input) {
        if (child.Name() == "Robot") {
            for (auto& grandchild : child) {
                if (grandchild.Name() == "Agent") {
                    std::string type = grandchild.Read("type", true, "", "The Agent class name.");
                    if (type == "coordinator")
                        coordinatorNode = &grandchild;
                }
            }
        }
    }

    SECTION("Behaviors TESTS: CoordinatorTest") {
        // Run the coordinator tests
        auto robot = problem->GetRobot("coordinator");
        coordinatorTest = new CoordinatorTest(robot, *coordinatorNode);

        std::cout << "\nBehaviors TESTS: CoordinatorTests" << std::endl;

        // TestInitialize
        auto initTestResult = coordinatorTest->TestInitialize();
        std::cout << initTestResult.second << "\n";
        CHECK(initTestResult.first == true);

        // TestStep
        auto stepTestResult = coordinatorTest->TestStep();
        std::cout << stepTestResult.second << "\n";
        CHECK(stepTestResult.first == true);

        // TestUninitialize
        auto uninitTestResult = coordinatorTest->TestUninitialize();
        std::cout << uninitTestResult.second << "\n";
        CHECK(uninitTestResult.first == true);
    }

    delete problem;
}
