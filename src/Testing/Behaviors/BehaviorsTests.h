#ifndef PPL_BEHAVIORS_TESTS_H_
#define PPL_BEHAVIORS_TESTS_H_

#include "Testing/TestBaseObject.h"
#include "Utilities/XMLNode.h"
#include "Testing/Behaviors/Agents/CoordinatorTest.h"
#include "MPProblem/MPProblem.h"
#include "Simulator/Simulation.h"

class BehaviorsTests : public TestBaseObject {
  public:
    ///@name LocalTypes
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    BehaviorsTests();

    BehaviorsTests(MPProblem* _problem, const std::string& _filename);

    virtual ~BehaviorsTests();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  private:
    ///@name Test objects
    ///@{

    MPProblem* m_problem{nullptr};
    std::string m_xmlFilename;
    XMLNode* m_coordinatorNode{nullptr};
    CoordinatorTest* m_coordinatorTest{nullptr};

    ///@}
};

/*--------------------------- Construction ---------------------------*/

BehaviorsTests::
BehaviorsTests() {}

BehaviorsTests::
BehaviorsTests(MPProblem* _problem, const std::string& _filename) :
               m_problem(_problem), m_xmlFilename(_filename) {}

BehaviorsTests::
~BehaviorsTests() {}
/*----------------------------- Interface ----------------------------*/

typename BehaviorsTests::TestResult
BehaviorsTests::
RunTest() {
  bool passed = true;
  std::string message = "";
  int total = 0;
  int numPassed = 0;

  message = message + "Running test for Coordinator...\n";

  // Create the simulation object needed for coordinator initialization
  Simulation::Create(m_problem);

  // Get the coordinator node from the XML file
  XMLNode input(m_xmlFilename, "Problem");

  for(auto& child : input) {
    if (child.Name() == "Robot") {
      for (auto& grandchild : child) {
        if (grandchild.Name() == "Agent") {
          std::string type = grandchild.Read("type", true, "", "The Agent class name.");
          if (type == "coordinator")
            m_coordinatorNode = &grandchild;
        }
      }
    }
  }

  // Run the coordinator tests
  auto robot = m_problem->GetRobot("coordinator");
  m_coordinatorTest = new CoordinatorTest(robot, *m_coordinatorNode);
  auto result = m_coordinatorTest->RunTest();

  // Track how many tests pass
  total++;
  if (result.first) {
    message = message + "PASSED!\n";
    numPassed++;
  } else {
    message = message + "FAILED :(\n";
  }

  passed = passed and result.first;
  message = message + result.second;

  message = message + "\nCOMPLETED Behaviors TESTS\n"
                      "Total: " + std::to_string(total) + "\n"
                      "Passed: " + std::to_string(numPassed) + "\n"
                      "Failed: " + std::to_string(total - numPassed) + "\n\n";

  delete m_coordinatorTest;
  return std::make_pair(passed, message);
}

/*--------------------------------------------------------------------*/

#endif
