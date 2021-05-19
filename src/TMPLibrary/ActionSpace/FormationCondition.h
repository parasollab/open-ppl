#ifndef PPL_FORMATION_H_
#define PPL_FORMATION_H_

#include "Condition.h"

class RobotGroup;
class TMPLibrary;

class FormationCondition : public Condition {
  public:
    ///@name Local Types
    ///@{

    typedef Condition::State State;

    ///@}
    ///@name Construction
    ///@{

    FormationCondition();

    FormationCondition(XMLNode& _node, TMPLibrary* _tmpLibrary);

    ~FormationCondition();

    ///@}
    ///@name Interface
    ///@{

    virtual RobotGroup* Satisfied(const State& _state) const override;

    ///@}

  private:
    ///@name Helper Functions
    ///@{

    bool CheckRequirements(RobotGroup* _group) const;

    ///@}
    ///@name Internal State
    ///@{

    std::vector<std::string> m_requiredTypes;

    ///@}
};

#endif
