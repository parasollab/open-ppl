#ifndef ITERATION_COUNT_EVALUATOR_H_
#define ITERATION_COUNT_EVALUATOR_H_

#include "MapEvaluatorMethod.h"

#include <limits>

#include "Utilities/MetricUtils.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/RapidCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/SpheresCollisionDetection.h"


////////////////////////////////////////////////////////////////////////////////
/// Returns true once a minimum number of iterations has been achieved.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class IterationCountEvaluator : public MapEvaluatorMethod<MPTraits> {

  public:
    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename RoadmapType::GraphType      GraphType;
    typedef typename RoadmapType::VID            VID;

    ///@name Construction
    ///@{

  IterationCountEvaluator(const string& _vcLabel = "", const double minDist = 5);

  IterationCountEvaluator(XMLNode& _node);

    virtual ~IterationCountEvaluator() = default;

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
    bool TestCfg(CfgType &_cfg);

    ///@name Internal State
    ///@{

    unsigned int m_minCount{0};
    unsigned int m_currentCount{0};
    bool m_minAchieved{false};

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
IterationCountEvaluator<MPTraits>::
IterationCountEvaluator(const string& _vcLabel, const double minCount) :
                          MapEvaluatorMethod<MPTraits>(), m_minCount(minCount) {
  this->SetName("IterationCountEvaluator");
}


template <typename MPTraits>
IterationCountEvaluator<MPTraits>::
IterationCountEvaluator(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("IterationCountEvaluator");
  m_minCount = _node.Read("minCount", true, m_minCount,
                          "Number of ME calls before returning true.");
}

/*------------------------- MPBaseObject Interface ---------------------------*/

template <typename MPTraits>
void
IterationCountEvaluator<MPTraits>::
Initialize() {
  m_currentCount = 0;
  m_minAchieved = false;
}

/*---------------------- MapEvaluatorMethod Interface ------------------------*/

template <typename MPTraits>
bool
IterationCountEvaluator<MPTraits>::
operator()() {
  if(!m_minAchieved)
    m_minAchieved = (++m_currentCount >= m_minCount);

  return m_minAchieved;
}


/*----------------------------------------------------------------------------*/

#endif
