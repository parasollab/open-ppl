#ifndef PPL_AGENT_TEST_H_
#define PPL_AGENT_TEST_H_

#include "Behaviors/Agents/Agent.h"
#include "Testing/TestBaseObject.h"

class Robot;
class XMLNode;

class AgentTest : virtual public Agent, 
                  public TestBaseObject {

  public:
    ///@name LocalTypes
    ///@{

    typedef std::pair<bool,std::string> TestResult;

    ///@}
    ///@name Construction
    ///@{

    AgentTest(Robot* const _r);

    AgentTest(Robot* const _r, XMLNode& _node);

    AgentTest(Robot* const _r, const Agent& _a);

    virtual ~AgentTest();

    ///@}
    ///@name Interface
    ///@{

    // TODO do I need this, or just the individual tests

    virtual TestResult RunTest();

    ///@}

  protected:
    ///@name Helper test functions
    ///@{

    TestResult TestSetTask();

    TestResult TestResetStartConstraint();

    TestResult TestInitialize();

    TestResult TestStep();

    TestResult TestUninitialize();

    TestResult TestMinimumSteps();

    TestResult TestProximityCheck();

    TestResult TestHalt();

    TestResult TestPauseAgent();

    ///@}
};

/*--------------------------- Construction ---------------------------*/
AgentTest::
AgentTest(Robot* const _r) : Agent(_r) {}

AgentTest::
AgentTest(Robot* const _r, XMLNode& _node) : Agent(Robot* const _r, XMLNode& _node) {}

AgentTest::
AgentTest(Robot* const _r, const Agent& _a) : Agent(Robot* const _r, const Agent& _a) {}

AgentTest::
~AgentTest() {}

/*----------------------- Helper test functions -----------------------*/
typename AgentTest::TestResult
AgentTest::
TestSetTask() {
  bool passed = true;
  std::string message = "";

  return std::make_pair(passed, message);
}

typename AgentTest::TestResult
AgentTest::
TestResetStartConstraint() {
  bool passed = true;
  std::string message = "";

  return std::make_pair(passed, message);
}

typename AgentTest::TestResult
AgentTest::
TestInitialize() {
  bool passed = true;
  std::string message = "";

  return std::make_pair(passed, message);
}

typename AgentTest::TestResult
AgentTest::
TestStep() {
  bool passed = true;
  std::string message = "";

  return std::make_pair(passed, message);
}

typename AgentTest::TestResult
AgentTest::
TestUninitialize() {
  bool passed = true;
  std::string message = "";

  return std::make_pair(passed, message);
}

typename AgentTest::TestResult
AgentTest::
TestMinimumSteps() {
  bool passed = true;
  std::string message = "";

  return std::make_pair(passed, message);
}

typename AgentTest::TestResult
AgentTest::
TestProximityCheck() {
  bool passed = true;
  std::string message = "";

  return std::make_pair(passed, message);
}

typename AgentTest::TestResult
AgentTest::
TestHalt() {
  bool passed = true;
  std::string message = "";

  return std::make_pair(passed, message);
}

typename AgentTest::TestResult
AgentTest::
TestPauseAgent() {
  bool passed = true;
  std::string message = "";

  return std::make_pair(passed, message);
}

#endif
