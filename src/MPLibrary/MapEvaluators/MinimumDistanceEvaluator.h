#ifndef MINIMUM_DISTANCE_EVALUATOR_H_
#define MINIMUM_DISTANCE_EVALUATOR_H_

#include "MapEvaluatorMethod.h"

#include <limits>

#include "Utilities/MetricUtils.h"


////////////////////////////////////////////////////////////////////////////////
/// This evaluator returns true once a minimum distance (using the provided
/// distance metric) has been reached in any roadmap configuration, relative to
/// the first (root) configuration in the roadmap.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MinimumDistanceEvaluator : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename MPTraits::CfgType     CfgType;
    typedef typename RoadmapType::VID      VID;

    ///@}
    ///@name Construction
    ///@{

    MinimumDistanceEvaluator(const double _timeout = 10, const string _dmLabel = "",
                      const double _minDist = 1.0);

    MinimumDistanceEvaluator(XMLNode& _node);

    virtual ~MinimumDistanceEvaluator() = default;

    ///@}
    ///@name MPBaseObject Interface
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluatorMethod Interface
    ///@{

    virtual bool operator()() override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    string     m_dmLabel;        ///< Distance metric label
    double     m_minDist = 1.0;  ///< Minimum distance for maximum time
    bool       m_minimumAchieved = false; ///< flag for faster check
    VID        m_lastNode = 0;   ///< last node of last operation, to save time

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MinimumDistanceEvaluator<MPTraits>::
MinimumDistanceEvaluator(const double _timeout, const string _dmLabel,
                  const double _minDist) : MapEvaluatorMethod<MPTraits>(),
                                  m_dmLabel(_dmLabel), m_minDist(_minDist) {
  this->SetName("MinimumDistanceEvaluator");
}


template <typename MPTraits>
MinimumDistanceEvaluator<MPTraits>::
MinimumDistanceEvaluator(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("MinimumDistanceEvaluator");
  m_dmLabel = _node.Read("dmLabel", true, "", "Distance Metric");
  m_minDist = _node.Read("minDist", true, 1.0, .0,
      std::numeric_limits<double>::max(),
      "Minimum distance to not end strategy.");
}

/*------------------------- MPBaseObject Interface ---------------------------*/

template <typename MPTraits>
void
MinimumDistanceEvaluator<MPTraits>::
Initialize() {
  m_minimumAchieved = false;
  m_lastNode = 0;
}

/*---------------------- MapEvaluatorMethod Interface ------------------------*/

template <typename MPTraits>
bool
MinimumDistanceEvaluator<MPTraits>::
operator()() {
  if(!m_minimumAchieved) {
    auto const graph = this->GetRoadmap()->GetGraph();
    auto const dm = this->GetDistanceMetric(m_dmLabel);
    const CfgType& root = graph->GetVertex(0);
    for( ; m_lastNode < graph->get_num_vertices(); ++m_lastNode)
      if(dm->Distance(root, graph->GetVertex(m_lastNode)) > m_minDist)
        m_minimumAchieved = true;
  }

  return m_minimumAchieved;
}

/*----------------------------------------------------------------------------*/

#endif
