#include "PlacementMethod.h"

PlacementMethod::
PlacementMethod(MPProblem* _problem) : m_problem(_problem) {}

void
PlacementMethod::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library){
  _solution->AddInteractionTemplate(_it);
}

std::string
PlacementMethod::
GetLabel(){
  return m_label;
}
