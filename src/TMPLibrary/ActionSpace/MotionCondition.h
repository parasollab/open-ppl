#ifndef PPL_MOTION_CONDITION_H_
#define PPL_MOTION_CONDITION_H_

#include "Condition.h"

class Constraint;
class RobotGroup;
class TMPLibrary;

class MotionCondition : public Condition {

  public: 
    ///@name Local Types
    ///@{

    typedef Condition::State State;

    ///@}
    ///@name Construction
    ///@{

    MotionCondition();

    MotionCondition(XMLNode& _node, TMPLibrary* _tmpLibrary);

    ~MotionCondition();

    ///@}
    ///@name Interface
    ///@{

    virtual RobotGroup* Satisfied(const State& _state) const override;

    ///@}
  private:
    ///@name Internal State
    ///@{

    /// Set of CSpace and Workspace constraints
    std::vector<std::unique_ptr<Constraint>> m_constraints;

    ///@}

};

#endif
