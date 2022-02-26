#ifndef PPL_COORDINATOR_TEST_H_
#define PPL_COORDINATOR_TEST_H_

#include "Behaviors/Agents/Coordinator.h"
#include "Testing/Behaviors/Agents/AgentTest.h"
#include "Testing/TestBaseObject.h"

class CoordinatorTest : public Coordinator,
                        public AgentTest {

  public: 
  
    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    CoordinatorTest(Robot* const _r);

    CoordinatorTest(Robot* const _r, XMLNode& _node);

    virtual std::unique_ptr<Agent> Clone(Robot* const _r) const;

    ~CoordinatorTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}
    ///@name Agent Interface
    ///@{

    virtual void Initialize();

    virtual void Step(const double _dt);

    virtual void Uninitialize();

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
CoordinatorTest(Robot* const _r) : Coordinator(_r), AgentTest(_r) {}

CoordinatorTest:: 
CoordinatorTest(Robot* const _r, XMLNode& _node) : Coordinator(_r, _node),
                                                   AgentTest(_r, _node) {}

std::unique_ptr<Agent> 
CoordinatorTest::
Clone(Robot* const _r) const {
  return Coordinator::Clone(_r);
}

CoordinatorTest::
~CoordinatorTest() {}

/*--------------------------- Agent Interface ------------------------*/

void
CoordinatorTest::
Initialize() {
  Coordinator::Initialize();
}

void
CoordinatorTest::
Step(const double _dt) {
  Coordinator::Step(_dt);
}

void
CoordinatorTest::
Uninitialize() {
  Coordinator::Uninitialize();
}

/*----------------------------- Interface ----------------------------*/

typename CoordinatorTest::TestResult
CoordinatorTest::
RunTest() {
  bool passed = true;
  std::string message = "";

  auto result = TestInitialize();
  passed = passed and result.first;
  message = message + result.second;

  result = TestStep();
  passed = passed and result.first;
  message = message + result.second;

  result = TestUninitialize();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed, message);
}

/*--------------------- Test Interface Functions ---------------------*/

typename CoordinatorTest::TestResult
CoordinatorTest::
TestInitialize() {
  bool passed = true;
  std::string message = "";

  if (passed) {
    message = "Initialize::PASSED!\n";
  } else {
    message = "Initialize::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

typename CoordinatorTest::TestResult
CoordinatorTest::
TestStep() {
  bool passed = true;
  std::string message = "";

  if (passed) {
    message = "Step::PASSED!\n";
  } else {
    message = "Step::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

typename CoordinatorTest::TestResult
CoordinatorTest::
TestUninitialize() {
  bool passed = true;
  std::string message = "";

  if (passed) {
    message = "Uninitialize::PASSED!\n";
  } else {
    message = "Uninitialize::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

/*--------------------------------------------------------------------*/
#endif
