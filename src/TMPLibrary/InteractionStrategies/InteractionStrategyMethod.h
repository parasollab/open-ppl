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

    ///@param _interaction The interaction to plan.
    ///@param _start Input as the start state. Modified to reflect the
    ///              output state.
    ///@return bool representing if successful or not.
    virtual bool operator()(Interaction* _interaction, State& _start);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    std::string m_sgLabel;

    ///@}

};
#endif
