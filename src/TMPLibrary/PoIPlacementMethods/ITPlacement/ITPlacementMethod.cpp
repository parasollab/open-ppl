#include "ITPlacementMethod.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"

ITPlacementMethod::
ITPlacementMethod(MPProblem* _problem) : m_problem(_problem) {}

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
