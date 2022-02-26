#ifndef PPL_AGENT_TEST_H_
#define PPL_AGENT_TEST_H_

#include "Behaviors/Agents/Agent.h"
#include "Testing/TestBaseObject.h"

class Robot;
class XMLNode;

class AgentTest : public Agent, 
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

    virtual TestResult RunTest() = 0;

    ///@}

  protected:

    ///@name Simulation Interface Test Functions
    ///@{

    virtual TestResult TestInitialize() = 0;

    virtual TestResult TestStep() = 0;

    virtual TestResult TestUninitialize() = 0;

    ///@}
};

/*--------------------------- Construction ---------------------------*/
AgentTest::
AgentTest(Robot* const _r) : Agent(_r) {}

AgentTest::
AgentTest(Robot* const _r, XMLNode& _node) : Agent(_r, _node) {}

AgentTest::
AgentTest(Robot* const _r, const Agent& _a) : Agent(_r, _a) {}

AgentTest::
~AgentTest() {}

/*--------------------------------------------------------------------*/
#endif
