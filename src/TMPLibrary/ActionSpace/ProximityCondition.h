#ifndef PPL_PROXIMITY_CONDITION_H_
#define PPL_PROXIMITY_CONDITION_H_

#include "Condition.h"

class RobotGroup;
class TMPLibrary;

class ProximityCondition : public Condition {
  public:
    ///@name Local Types
    ///@{

    typedef Condition::State State;

    ///@}
    ///@name Construction
    ///@{

    ProximityCondition();

    ProximityCondition(XMLNode& _node, TMPLibrary* _library);

    ~ProximityCondition();

    ///@}
    ///@name Interface
    ///@{

    virtual RobotGroup* Satisfied(const State& _state) const override;

    ///@}

  private:
    ///@name Helper Functions
    ///@{

    void ParseXML(XMLNode& _node);

    ///@}
    ///@name Internal State
    ///@{

    std::unique_ptr<Boundary> m_boundary;

    ///@}

};

#endif
