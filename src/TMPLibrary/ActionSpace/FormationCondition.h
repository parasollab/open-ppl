#ifndef PPL_FORMATION_CONDITION_H_
#define PPL_FORMATION_CONDITION_H_

#include "Condition.h"
#include "Transformation.h"

class Formation;
class RobotGroup;
class TMPLibrary;

class FormationCondition : public Condition {
  public:
    ///@name Local Types
    ///@{

    typedef Condition::State State;

    struct Role {
      bool leader = true;
      std::string name;
      std::string type;
      std::string referenceRole;
      size_t referenceBody;
      size_t dependentBody;
      Transformation transformation;
    };

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

    const std::vector<std::string> GetRoles() const;
  
    Formation* GenerateFormation(std::unordered_map<std::string,Robot*>& _roleMap);

    ///@}

  private:
    ///@name Helper Functions
    ///@{

    bool CheckRequirements(RobotGroup* _group) const;

    std::vector<double> ParseVectorString(std::string _s);

    ///@}
    ///@name Internal State
    ///@{

    std::vector<std::string> m_requiredTypes;

    std::unordered_map<std::string,Role> m_roles;

    ///@}
};

#endif
