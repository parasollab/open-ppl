#ifndef PPL_COORDINATOR_TEST_H_
#define PPL_COORDINATOR_TEST_H_

#include "Behaviors/Agents/Coordinator.h"
#include "Testing/Behaviors/Agents/AgentTest.h"

template <typename MPTraits>
class CoordinatorTest : virtual public Coordinator<MPTraits>,
                        public AgentTest<MPTraits> {

  public: 
  
    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    CoordinatorTest(Robot* const _r);

    CoordinatorTest(Robot* const _r, XMLNode& _node);

    ~CoordinatorTest();

    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult TestInitialize() override;

    virtual TestResult TestStep() override;

    virtual TestResult TestUninitialize() override;

    ///@}
};

/*--------------------------- Construction ---------------------------*/

CoordinatorTest::
CoordinatorTest(Robot* const _r) : Coordinator(_r) {}

CoordinatorTest:: 
CoordinatorTest(Robot* const _r, XMLNode& _node) : Agent(_r, _node),
                                                   Coordinator(_r, _node) {}

CoordinatorTest::
~CoordinatorTest() {}

/*--------------------- Test Interface Functions ---------------------*/

typename CoordinatorTest::TestResult
CoordinatorTest::
TestInitialize() {
  bool passed = true;
  std::string message = "";

  return std::make_pair(passed, message);
}

typename CoordinatorTest::TestResult
CoordinatorTest::
TestStep() {
  bool passed = true;
  std::string message = "";

  return std::make_pair(passed, message);
}

typename CoordinatorTest::TestResult
CoordinatorTest::
TestUninitialize() {
  bool passed = true;
  std::string message = "";

  return std::make_pair(passed, message);
}

/*--------------------------------------------------------------------*/
#endif
