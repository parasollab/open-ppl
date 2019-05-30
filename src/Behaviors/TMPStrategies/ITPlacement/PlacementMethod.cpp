#include "PlacementMethod.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/TMPStrategies/TMPStrategyMethod.h"

PlacementMethod::
PlacementMethod(MPProblem* _problem) : m_problem(_problem) {}

std::unique_ptr<PlacementMethod>
PlacementMethod::
Clone(){
	return std::unique_ptr<PlacementMethod>(new PlacementMethod(*this));	
}

void
PlacementMethod::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library, TMPStrategyMethod* _tmpMethod){
  //_solution->AddInteractionTemplate(_it);
  std::cout << "Base Type" << std::endl;
}

std::string
PlacementMethod::
GetLabel(){
  return m_label;
}
