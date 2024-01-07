#include "MinimumDistanceEvaluator.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------ Construction --------------------------------*/

MinimumDistanceEvaluator::MinimumDistanceEvaluator(const double _timeout,
                                                   const string _dmLabel,
                                                   const double _minDist)
    : MapEvaluatorMethod(), m_dmLabel(_dmLabel), m_minDist(_minDist) {
  this->SetName("MinimumDistanceEvaluator");
}

MinimumDistanceEvaluator::MinimumDistanceEvaluator(XMLNode& _node)
    : MapEvaluatorMethod(_node) {
  this->SetName("MinimumDistanceEvaluator");
  m_dmLabel = _node.Read("dmLabel", true, "", "Distance Metric");
  m_minDist =
      _node.Read("minDist", true, 1.0, .0, std::numeric_limits<double>::max(),
                 "Minimum distance to not end strategy.");
}

/*------------------------- MPBaseObject Interface ---------------------------*/

void MinimumDistanceEvaluator::Initialize() {
  m_minimumAchieved = false;
  m_lastNode = 0;
}

/*---------------------- MapEvaluatorMethod Interface ------------------------*/

bool MinimumDistanceEvaluator::operator()() {
  if (!m_minimumAchieved) {
    auto const graph = this->GetRoadmap();
    auto const dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
    const Cfg& root = graph->GetVertex(0);
    for (; m_lastNode < graph->get_num_vertices(); ++m_lastNode)
      if (dm->Distance(root, graph->GetVertex(m_lastNode)) > m_minDist)
        m_minimumAchieved = true;
  }

  return m_minimumAchieved;
}

/*----------------------------------------------------------------------------*/
