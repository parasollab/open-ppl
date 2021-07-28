#ifndef PPL_PROXIMITY_CONDITION_H_
#define PPL_PROXIMITY_CONDITION_H_

#include "Condition.h"

class Boundary;
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
    ///@name Accessors

    Boundary* GetBoundary(State _state = State());

    ///@}
  private:

    ///@name Helper Functions
    ///@{

    void ParseXML(XMLNode& _node);

    std::vector<double> ComputeCompositeCenter(const State& _state) const;

    ///@}
    ///@name Internal State
    ///@{

    double m_threshold;

    std::unique_ptr<Boundary> m_boundary;

    ///@}

};

#endif
