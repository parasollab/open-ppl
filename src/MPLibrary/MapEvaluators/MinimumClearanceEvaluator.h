#ifndef MINIMUM_CLEARANCE_EVALUATOR_H_
#define MINIMUM_CLEARANCE_EVALUATOR_H_

#include "MapEvaluatorMethod.h"

#include <limits>

#include "Utilities/MetricUtils.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/RapidCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/SpheresCollisionDetection.h"


////////////////////////////////////////////////////////////////////////////////
/// An assembly planning map evaluator that checks for a minimum distance as
/// returned by a CDInfo object from the provided validity checker. When used
/// with a "Specific Body" validity checker, this is checking the bodies how the
/// validity checker is set up (so it would check if the set parts are moved far
/// enough away from all of the other parts of the robot.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MinimumClearanceEvaluator : public MapEvaluatorMethod<MPTraits> {

  public:
    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename RoadmapType::GraphType      GraphType;
    typedef typename RoadmapType::VID            VID;

    ///@name Construction
    ///@{

  MinimumClearanceEvaluator(const string& _vcLabel = "", const double minDist = 5);

  MinimumClearanceEvaluator(XMLNode& _node);

    virtual ~MinimumClearanceEvaluator() = default;

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

    size_t m_lastVid = 0;                 ///< last tested node
    string m_vcLabel;                     ///< validity checker label
    double m_minDist;                     ///< minimum distance between the bodies of the robot

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MinimumClearanceEvaluator<MPTraits>::
MinimumClearanceEvaluator(const string& _vcLabel,
    const double minDist) : MapEvaluatorMethod<MPTraits>(),
    m_vcLabel(_vcLabel), m_minDist(minDist) {
  this->SetName("MinimumClearanceEvaluator");
}


template <typename MPTraits>
MinimumClearanceEvaluator<MPTraits>::
MinimumClearanceEvaluator(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("MinimumClearanceEvaluator");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity checker label");
  m_minDist = _node.Read("minDist", false, 5.0, 0.0, 100.0,
          "Number of Expansions per iteration");
}

/*------------------------- MPBaseObject Interface ---------------------------*/

template <typename MPTraits>
void
MinimumClearanceEvaluator<MPTraits>::
Initialize() {
  m_lastVid = 0;
}

/*---------------------- MapEvaluatorMethod Interface ------------------------*/

template <typename MPTraits>
bool
MinimumClearanceEvaluator<MPTraits>::
operator()() {
  auto graph = this->GetRoadmap()->GetGraph();
  const size_t numVerts = graph->get_num_vertices();

  // start at the last position (m_lastVid), because all previous nodes has been tested
  for ( ; m_lastVid < numVerts; ++m_lastVid) {
    CfgType& cfg = graph->GetVertex(m_lastVid);
    if (!TestCfg(cfg))
      continue;
    else
      return true;
  }
  return false;
}

template <typename MPTraits>
bool
MinimumClearanceEvaluator<MPTraits>::
TestCfg(CfgType& _cfg) {
  auto vc = this->GetValidityChecker(m_vcLabel);
  string callee("MinimumClearanceEvaluator::TestCfg");

  CDInfo cdInfo(true); // Need full collision info.
  if (!vc->IsValid(_cfg, cdInfo, callee))
    return false;

  if (cdInfo.m_minDist < m_minDist) {
    if(this->m_debug)
      std::cout << this->GetNameAndLabel() << "Distance check failed. "
                "Dist = " << cdInfo.m_minDist << endl;
    return false;
  }
  else {
    if(this->m_debug) {
      if(cdInfo.m_nearestObstIndex == -1 && cdInfo.m_selfClearance.empty())
        std::cout << "Warning: the distance evaluated appears to be set to"
              " default. This is still considered success for now" << std::endl;
      std::cout << this->GetNameAndLabel() << "Distance check was valid with: "
                << cdInfo.m_minDist << endl;
    }
    return true;
  }
}

/*----------------------------------------------------------------------------*/

#endif
