#ifndef PPL_COORDINATOR_TEST_H_
#define PPL_COORDINATOR_TEST_H_

#include "Behaviors/Agents/Coordinator.h"   //src
#include "AgentTest.h"

class CoordinatorTest : public Coordinator, public AgentTest {

  public: 
  
    ///@name Local Types
    ///@{

    typedef std::pair<bool,std::string> TestResult;

    ///@}
    ///@name Construction
    ///@{

    CoordinatorTest(Robot* const _r);

    CoordinatorTest(Robot* const _r, XMLNode& _node);

    virtual std::unique_ptr<Agent> Clone(Robot* const _r) const;

    ~CoordinatorTest();

    ///@}
    ///@name Agent Interface
    ///@{

    virtual void Initialize();

    virtual void Step(const double _dt);

    virtual void Uninitialize();

    ///@}

  public:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult TestInitialize() override;

    virtual TestResult TestStep() override;

    virtual TestResult TestUninitialize() override;

    ///@}
};

/*--------------------------- Construction ---------------------------*/

CoordinatorTest::CoordinatorTest(Robot* const _r) : Coordinator(_r), AgentTest(_r) {}

CoordinatorTest::CoordinatorTest(Robot* const _r, XMLNode& _node) : Coordinator(_r, _node),
                                  AgentTest(_r, _node) {}

std::unique_ptr<Agent> 
CoordinatorTest::Clone(Robot* const _r) const {
  return Coordinator::Clone(_r);
}

CoordinatorTest::~CoordinatorTest() {}

/*--------------------------- Agent Interface ------------------------*/

void CoordinatorTest::Initialize() {
  Coordinator::Initialize();
}

void CoordinatorTest::Step(const double _dt) {
  Coordinator::Step(_dt);
}

void CoordinatorTest::Uninitialize() {
  Coordinator::Uninitialize();
}

/*--------------------- Test Interface Functions ---------------------*/

typename CoordinatorTest::TestResult CoordinatorTest::TestInitialize() {
  bool passed = true;
  std::string message = "";

  // Check that the number of child agents is initially zero and then increases
  if (this->m_childAgents.size() != 0) {
    passed = false;
    std::cout << "\n\tThe child agents are not correctly initialized." << std::endl;
  }

  Initialize();

  if (passed and (this->m_childAgents.size() <= 0)) {
    passed = false;
    std::cout << "\n\tThe Coordinator has the incorrect number of child agents." << std::endl;
  }

  message = "\tFINISHED TestInitialize";
  return std::make_pair(passed, message);
}

typename CoordinatorTest::TestResult CoordinatorTest::TestStep() {
  bool passed = true;
  std::string message = "";

  // Calling Step will solve the problem, so better left to strategy tests

  message = "\tFINISHED TestStep";
  return std::make_pair(passed, message);
}

typename CoordinatorTest::TestResult CoordinatorTest::TestUninitialize() {
  bool passed = true;
  std::string message = "";

  // Check that the solution and library are deleted
  Uninitialize();
  if (m_solution or m_library) {
    passed = false;
    std::cout << "\n\tSolution or library was not deleted." << std::endl;
  }

  message = "\tFINISHED TestUninitialize";
  return std::make_pair(passed, message);
}

/*--------------------------------------------------------------------*/
#endif
