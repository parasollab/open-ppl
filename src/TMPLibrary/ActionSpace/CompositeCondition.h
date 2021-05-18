#ifndef PPL_COMPOSITE_CONDITION_H_
#define PPL_COMPOSITE_CONDITION_H_

#include "Condition.h"

class GroupRobot;

class CompositeCondition : public Condition {
  public:
    ///@name Local Types
    ///@{

    typedef Condition::State State;

    ///@}
    ///@name Constructors
    ///@{

    CompositeCondition();

    CompositeCondition(XMLNode& _node);

    ~CompositeCondition();

    ///@}
    ///@name Interface
    ///@{

    GroupRobot* Satisfied(const State& _state);

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
