#ifndef IT_CONSTRUCTOR_H_
#define IT_CONSTRUCTOR_H_


#include "Behaviors/Agents/Agent.h"

#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/PMPL.h"

#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Robot.h"

#include "TMPLibrary/TMPTools/InteractionTemplate.h"

class ITConstructor{
  public:
    ///@name Local Types
    ///@{

    

    ///@}
    ///@name Construction
    ///@{

    ITConstructor(MPLibrary* _library,
                  std::vector<Agent*> _memberAgents,
                  Robot* _superRobot);

    ~ITConstructor() = default;

    ///@}
    ///@name Call Method
    ///@{

    void ConstructIT(InteractionTemplate* _it);

    ///@}

  private:
    MPLibrary* m_library;
    MPProblem* m_problem;
    std::shared_ptr<MPProblem> m_problemCopy;
    std::vector<Agent*> m_memberAgents;
    Robot* m_superRobot;
    bool m_debug;


};
#endif
