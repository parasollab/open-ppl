#ifndef PPL_BEHAVIORS_TESTS_H_
#define PPL_BEHAVIORS_TESTS_H_

#include "Utilities/XMLNode.h"      //src
#include "Agents/CoordinatorTest.h"
#include "MPProblem/MPProblem.h"    //src
#include "Simulator/Simulation.h"   //src

class BehaviorsTests {
  public:
    ///@name LocalTypes
    ///@{total
 
    typedef std::pair<bool,std::string> TestResult;

    ///@}
    ///@name Construction
    ///@{

    BehaviorsTests();

    BehaviorsTests(MPProblem* _problem, const std::string& _filename);

    virtual ~BehaviorsTests();

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

BehaviorsTests::BehaviorsTests() {}

BehaviorsTests::BehaviorsTests(MPProblem* _problem, const std::string& _filename) :
                m_problem(_problem), m_xmlFilename(_filename) {}

BehaviorsTests::~BehaviorsTests() {}

/*--------------------------------------------------------------------*/

#endif
