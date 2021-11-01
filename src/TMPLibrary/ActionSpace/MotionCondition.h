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

    virtual void AssignRoles(std::unordered_map<std::string,Robot*>& _roleMap,
                             const State& _state) const override;

    ///@}
    ///@name Accessors
    ///@{

    const std::vector<std::pair<std::string,Constraint*>> GetConstraints() const;

    std::vector<Constraint*> GetConstraints(std::string _type) const;

    std::string GetRole(Constraint* _constraint) const;

    std::set<std::string> GetRoles() const;

    void ReCenter(const std::vector<double>& _t);

    Constraint* GetRoleConstraint(const std::string& _role) const;

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    ///@}
    ///@name Internal State
    ///@{

    /// Set of pairs of robot type and CSpace/Workspace constraints
    std::vector<std::pair<std::string,std::unique_ptr<Constraint>>> m_constraints;

    std::unordered_map<std::string,Constraint*> m_roleConstraints;

    std::vector<std::pair<std::string,std::unique_ptr<Constraint>>> m_translatedConstraints;

    std::unordered_map<Constraint*,std::string> m_roles;

    bool m_explicit{false}; ///< Flag indicating that constraints cannot be shifted.

    ///@}

};

#endif
