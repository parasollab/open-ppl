#ifndef _PPL_INTERACTION_STRATEGY_METHOD_H_
#define _PPL_INTERACTION_STRATEGY_METHOD_H_

#include "TMPLibrary/ActionSpace/Condition.h"
#include "TMPLibrary/TMPBaseObject.h"

class Interaction;

class InteractionStrategyMethod : public TMPBaseObject {

  public:
    ///@name Local Types
    ///@{

    typedef Condition::State State;

    ///@}
    ///@name Construction
    ///@{

    InteractionStrategyMethod() = default;

    InteractionStrategyMethod(XMLNode& _node);

    virtual ~InteractionStrategyMethod();

    ///@}
    ///@name Interface
    ///@{

    virtual bool operator()(Interaction* _interaction, const State& _start);

    ///@}

  private:

    ///@name Internal State
    ///@{

    ///@}

};
#endif
