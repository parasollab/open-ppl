#ifndef PPL_BEHAVIORS_TESTS_H_
#define PPL_BEHAVIORS_TESTS_H_

#include "Testing/TestBaseObject.h"
#include "Utilities/XMLNode.h"
#include "Testing/Behaviors/Agents/CoordinatorTest.h"

class BehaviorsTests : public TestBaseObject {
  public:
    ///@name LocalTypes
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    BehaviorsTests();

    BehaviorsTests(XMLNode& _node);

    virtual ~BehaviorsTests();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}
  
  private:

    ///@name Test objects
    ///@{

    CoordinatorTest* m_coordinatorTest{nullptr};

    ///@}
};

/*--------------------------- Construction ---------------------------*/

BehaviorsTests::
BehaviorsTests() {}

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
  m_coordinatorTest = new CoordinatorTest(nullptr);
  auto result = m_coordinatorTest->RunTest();

  total++;
  if (result.first) {
    message = message + "PASSED!\n";
    numPassed++;
  }
  else
    message = message + "FAILED :(\n";

  passed = passed and result.first;
  message = message + result.second;

  message = message + "\nCOMPLETED BEHAVIORS TESTS\n"
                      "Total: " + std::to_string(total) + "\n"
                      "Passed: " + std::to_string(numPassed) + "\n"
                      "Failed: " + std::to_string(total - numPassed) + "\n\n";

  return std::make_pair(passed, message);
}

/*--------------------------------------------------------------------*/

#endif