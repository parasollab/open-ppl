#include "PlacementMethod.h"

#include "Behaviors/Agents/Coordinator.h"

PlacementMethod::
PlacementMethod(MPProblem* _problem) : m_problem(_problem) {}

void
PlacementMethod::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library, Coordinator* _coordinator){
  //_solution->AddInteractionTemplate(_it);
  std::cout << "Base Type" << std::endl;
}

std::string
PlacementMethod::
GetLabel(){
  return m_label;
}
