#include "InteractionStrategyMethod.h"

/*--------------------------------------- Construction ---------------------------------*/

InteractionStrategyMethod::
InteractionStrategyMethod(XMLNode& _node) : TMPBaseObject(_node) {
  m_sgLabel = _node.Read("sgLabel", false, "", "StateGraph to use within strategy.");
}

InteractionStrategyMethod::
~InteractionStrategyMethod() { }

/*--------------------------------------- Interface ---------------------------------*/

bool 
InteractionStrategyMethod::
operator()(Interaction* _interaction, State& _start) {
  return false;
}

