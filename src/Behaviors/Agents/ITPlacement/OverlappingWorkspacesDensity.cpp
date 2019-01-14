#include "OverlappingWorkspacesDensity.h"

OverlappingWorkspacesDensity::
OverlappingWorkspacesDensity(MPProblem* _problem) : PlacementMethod(_problem) {}


OverlappingWorkspacesDensity::
OverlappingWorkspacesDensity(MPProblem* _problem, XMLNode& _node) : PlacementMethod(_problem) {
  m_label = _node.Read("label", true, "", "label for a fixed base it placement method");
}

void
OverlappingWorkspacesDensity::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library){
  _solution->AddInteractionTemplate(_it);
}
