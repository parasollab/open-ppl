#include "ITPlacementMethod.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/Solution/Plan.h"

ITPlacementMethod::
ITPlacementMethod(XMLNode& _node) : PoIPlacementMethod(_node) {}

std::unique_ptr<ITPlacementMethod>
ITPlacementMethod::
Clone(){
	return std::unique_ptr<ITPlacementMethod>(new ITPlacementMethod(*this));	
}

void
ITPlacementMethod::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution){
  //_solution->AddInteractionTemplate(_it);
  std::cout << "Base Type" << std::endl;
}

