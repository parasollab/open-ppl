#ifndef PMPL_MINIMUM_CLEARANCE_EVALUATOR_H_
#define PMPL_MINIMUM_CLEARANCE_EVALUATOR_H_

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
    typedef typename RoadmapType::VID            VID;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef std::vector<size_t>                  RobotFormation;


    ///@name Construction
    ///@{

    MinimumClearanceEvaluator(const string& _vcLabel = "",
        const double minDist = 5);

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

    double GetMinDist() { return m_minDist; }

  private:

    // TODO: These can probably be simplified into a templated function
    bool TestCfg(CfgType& _cfg);
    bool TestCfg(GroupCfgType& _cfg, const RobotFormation& _activeRobots = RobotFormation());

    ///@name Internal State
    ///@{

    size_t m_lastVid = 0; ///< last tested node
    string m_vcLabel;     ///< validity checker label
    double m_minDist;     ///< minimum distance between the bodies of the robot

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
  // If there's a group roadmap, then use that:
  GroupRoadmapType* const groupMap = this->GetGroupRoadmap();

  // TODO: This can probably be simplified into less duplicate code.
  if(groupMap) {
    const size_t numVerts = groupMap->get_num_vertices();

    // start at the last position (m_lastVid), because all previous nodes has been tested
    for ( ; m_lastVid < numVerts; ++m_lastVid) {
      if (!TestCfg(groupMap->GetVertex(m_lastVid), this->m_activeRobots))
        continue;
      else
        return true;
    }
    return false;
  }
  else {
    auto graph = this->GetRoadmap();
    const size_t numVerts = graph->get_num_vertices();

    // start at the last position (m_lastVid), because all previous nodes has been tested
    for ( ; m_lastVid < numVerts; ++m_lastVid) {
      if (!TestCfg(graph->GetVertex(m_lastVid)))
        continue;
      else
        return true;
    }
    return false;
  }
}

template <typename MPTraits>
bool
MinimumClearanceEvaluator<MPTraits>::
TestCfg(CfgType& _cfg) {
  auto vc = this->GetValidityChecker(m_vcLabel);
  string callee("MinimumClearanceEvaluator::TestCfg");

  CDInfo cdInfo(true); // Need full collision info.
  if(!vc->IsValid(_cfg, cdInfo, callee))
    return false;

  if (cdInfo.m_minDist < m_minDist) {
    if(this->m_debug)
      std::cout << this->GetNameAndLabel() << "Distance check failed. "
                "Dist = " << cdInfo.m_minDist << endl;
    return false;
  }
  else {
    if(this->m_debug) {
      std::cout << this->GetNameAndLabel() << "Distance check was valid with: "
                << cdInfo.m_minDist << endl;
    }
    return true;
  }
}


template <typename MPTraits>
bool
MinimumClearanceEvaluator<MPTraits>::
TestCfg(GroupCfgType& _cfg, const RobotFormation& _activeRobots) {
  if(_activeRobots.empty())
    throw RunTimeException(WHERE, "Must provide active robots for groups!");

  auto vc = this->GetValidityChecker(m_vcLabel);
  string callee("MinimumClearanceEvaluator::TestCfg");

  if(_cfg.GetGroupRoadmap() != this->GetGroupRoadmap())
    throw RunTimeException(WHERE, "The group roadmaps don't match!");

  CDInfo cdInfo(true); // Need full collision info.
  //if(!vc->IsValid(_cfg, cdInfo, callee, _activeRobots))
  if(!vc->IsValid(_cfg, cdInfo, callee))
    return false;

  const bool hasObstacles = this->GetEnvironment()->NumObstacles() > 0;
  const bool movingAllRobots =
          this->GetGroupTask()->GetRobotGroup()->Size() == _activeRobots.size();

  // Check clearance is valid; having obstacles or not moving all robots means
  // that the clearance value should not be the maximum.
  if(cdInfo.m_minDist == std::numeric_limits<double>::max()
      and (hasObstacles or !movingAllRobots))
    throw RunTimeException(WHERE, "Validity checker returned max value for "
        "minimum clearance! This is not okay unless using an "
        "empty environment and moving all robots simultaneously in _cfg!");

  if (cdInfo.m_minDist < m_minDist) {
    if(this->m_debug)
      std::cout << this->GetNameAndLabel() << "Distance check failed. "
                "Dist = " << cdInfo.m_minDist << endl;
    return false;
  }
  else {
    if(this->m_debug) {
      std::cout << this->GetNameAndLabel() << "Distance check was valid with: "
                << cdInfo.m_minDist << endl;
    }
    return true;
  }
}

/*----------------------------------------------------------------------------*/

#endif
