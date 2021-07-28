#ifndef PPL_MOTION_CONDITION_H_
#define PPL_MOTION_CONDITION_H_

#include "Condition.h"
#include "Transformation.h"

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
    ///@name Accessors
    ///@{

    const std::vector<std::pair<std::string,std::unique_ptr<Constraint>>>&
                    GetConstraints();

    std::vector<Constraint*> GetConstraints(std::string _type);

    std::string GetRole(Constraint* _constraint);

    std::set<std::string> GetRoles();

    std::vector<std::unique_ptr<Constraint>>&& GetTransformedConstraints(Transformation& _transform);
    ///@}
  private:
    ///@name Internal State
    ///@{

    /// Set of pairs of robot type and CSpace/Workspace constraints
    std::vector<std::pair<std::string,std::unique_ptr<Constraint>>> m_constraints;

    std::unordered_map<Constraint*,std::string> m_roles;
    ///@}

};

#endif
