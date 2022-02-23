#ifndef PPL_BEHAVIORS_TESTS_H_
#define PPL_BEHAVIORS_TESTS_H_

#include "Testing/TestBaseObject.h"
#include "Testing/Behaviorsy/Agents/CoordinatorTest.h"

class BehaviorsTests : public TestBaseObject {
  public:
    ///@name LocalTypes
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    BehaviorsTests();

    BehaviorsTests(const std::string& _xmlFile);

    virtual ~BehaviorsTests();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}
  
  private:

    // CoordinatorTest* m_coordinatorTest{nullptr};
};

/*--------------------------- Construction ---------------------------*/

BehaviorsTests::
BehaviorsTests() {
  // TODO
}

BehaviorsTests::
BehaviorsTests(const std::string& _xmlFile) {
  // TODO
}

BehaviorsTests::
~BehaviorsTests() {}
/*----------------------------- Interface ----------------------------*/

typename BehaviorsTests::TestResult
BehaviorsTests::
RunTest() {
  return TestResult(); // TODO
}

/*--------------------------------------------------------------------*/

#endif