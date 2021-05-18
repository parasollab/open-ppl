#include "InteractionStrategyMethod.h"

/*--------------------------------------- Construction ---------------------------------*/

InteractionStrategyMethod::
InteractionStrategyMethod(XMLNode& _node) : TMPBaseObject(_node) {

}

InteractionStrategyMethod::
~InteractionStrategyMethod() { }

/*--------------------------------------- Interface ---------------------------------*/

bool 
InteractionStrategyMethod::
operator()(Interaction* _interaction, const State& _start) {
  return false;
}

