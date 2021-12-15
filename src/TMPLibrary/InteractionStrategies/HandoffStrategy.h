#ifndef PPL_HANDOFF_STRATEGY_PPL_
#define PPL_HANDOFF_STRATEGY_PPL_

#include "GraspStrategy.h"

class HandoffStrategy : public GraspStrategy {

  public:
    ///@name Local Types
    ///@{

    typedef Condition::State State;

    ///@}
    ///@name Construction
    ///@{

    HandoffStrategy();

    HandoffStrategy(XMLNode& _node);

    ~HandoffStrategy();

    ///@}
    ///@name Interface
    ///@{

    virtual bool operator()(Interaction* _interaction, State& _start) override;

    ///@}

  protected:
    ///@name Helper Functions
    ///@{

    State GenerateTransitionState(Interaction* _interaction, const State& _previous, const size_t _next);

    ///@}
    ///@name Internal State
    ///@{


    ///@}

};

#endif
