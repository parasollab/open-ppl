#include "ITPlacementMethod.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"

ITPlacementMethod::
ITPlacementMethod(XMLNode& _node) : PoIPlacementMethod(_node) {}

std::unique_ptr<PlacementMethod>
ITPlacementMethod::
Clone(){
	return std::unique_ptr<PlacementMethod>(new ITPlacementMethod(*this));	
}

void
ITPlacementMethod::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library, TMPStrategyMethod* _tmpMethod){
  //_solution->AddInteractionTemplate(_it);
  std::cout << "Base Type" << std::endl;
}

std::string
ITPlacementMethod::
GetLabel(){
  return m_label;
}
