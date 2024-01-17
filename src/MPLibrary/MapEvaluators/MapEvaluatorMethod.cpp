#include "MapEvaluatorMethod.h"


MapEvaluatorMethod::
MapEvaluatorMethod(XMLNode& _node) : MPBaseObject(_node) {
}

/*----------------------------- Active Robots --------------------------------*/

void
MapEvaluatorMethod::
SetActiveRobots(const std::vector<size_t>& _activeRobots) {
  m_activeRobots = _activeRobots;
}


std::vector<size_t>
MapEvaluatorMethod::
GetActiveRobots() const {
  return m_activeRobots;
}

/*----------------------------------------------------------------------------*/
