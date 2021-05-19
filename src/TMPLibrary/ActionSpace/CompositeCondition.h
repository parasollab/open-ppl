#ifndef PPL_COMPOSITE_CONDITION_H_
#define PPL_COMPOSITE_CONDITION_H_

#include "Condition.h"

class RobotGroup;
class TMPLibrary;

class CompositeCondition : public Condition {
  public:
    ///@name Local Types
    ///@{

    typedef Condition::State State;

    ///@}
    ///@name Constructors
    ///@{

    CompositeCondition();

    CompositeCondition(XMLNode& _node, TMPLibrary* _tmpLibrary);

    ~CompositeCondition();

    ///@}
    ///@name Interface
    ///@{

    RobotGroup* Satisfied(const State& _state);

    ///@}
    ///@name Accessors
    ///@{
    ///@}

  private:
    ///@name Helper Functions
    ///@{

    virtual bool IsUnique() const override;

    ///@}
    ///@name Internal State
    ///@{

    std::vector<std::string> m_subconditions;

    ///@}
};

#endif
