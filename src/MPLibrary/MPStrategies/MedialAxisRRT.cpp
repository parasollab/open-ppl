#include "MedialAxisRRT.h"
#include "MPLibrary/MPLibrary.h"

MedialAxisRRT::
MedialAxisRRT() {
  this->SetName("MedialAxisRRT");
}


MedialAxisRRT::
MedialAxisRRT(XMLNode& _node) : BasicRRTStrategy(_node) {
  this->SetName("MedialAxisRRT");

  // Parse RRT parameters
  m_goalExtender = _node.Read("goalExtender", false, m_goalExtender,
      "Extender to use for goal extensions");

}

MedialAxisRRT::
~MedialAxisRRT() { }

/*------------------------- MPBaseObject Overrides ---------------------------*/

void
MedialAxisRRT::
Print(std::ostream& _os) const {
  BasicRRTStrategy::Print(_os);
  _os << "\tSampler: " << m_samplerLabel
      << "\n\tNeighborhood Finder: " << m_nfLabel
      << "\n\tExtender: " << m_exLabel
      << "\n\tConnection Method: " << m_ncLabel
      << "\n\tGoal check DM: " << m_goalDmLabel
      << "\n\tGrow Goals: " << m_growGoals
      << "\n\tGrowth Focus: " << m_growthFocus
      << "\n\tExpansion directions / trials: " << m_numDirections
      << " / " << m_disperseTrials
      << std::endl;
}


void
MedialAxisRRT::
TryGoalExtension(const VID _newVID, const Boundary* const _boundary) {
  if(!_boundary)
    throw RunTimeException(WHERE) << "Constraints which do not produce a "
                                  << "boundary are not supported.";

  // First check if _newVID is already in the goal region.
  auto g = this->GetRoadmap();
  const Cfg& cfg = g->GetVertex(_newVID);
  const bool inGoal = _boundary->InBoundary(cfg);
  if(inGoal) {
    if(this->m_debug)
      std::cout << "\tNode is already in this goal boundary." << std::endl;
    return;
  }

  // Get the nearest point to _newVID within this goal region.
  std::vector<double> data = cfg.GetData();
  _boundary->PushInside(data);
  Cfg target(cfg.GetRobot());
  target.SetData(data);

  // Check the nearest point to _newVID in each goal region. If it lies within
  // the goal threshold, try to extend towards it.
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_goalDmLabel);
  const double distance = dm->Distance(cfg, target),
               range = m_goalThreshold == 0.
                     ? this->GetMPLibrary()->GetExtender(m_goalExtender)->GetMaxDistance()
                     : m_goalThreshold;


  if(this->m_debug)
    std::cout << "\tNearest goal configuration is " << distance << " / "
              << range << " units away at " << target.PrettyPrint()
              << "."
              << std::endl;

  // If we are out of range, do not attempt to extend.
  if(distance > range) {
    if(this->m_debug)
      std::cout << "\tNot attempting goal extension." << std::endl;
    return;
  }

  // Try to extend towards the target.
  const VID extended = this->ExtendWithGoalExtender(_newVID, target);
  if(extended == INVALID_VID)
    return;

  // Check if we reached the goal boundary.
  const Cfg& extendedCfg = g->GetVertex(extended);
  const bool reached = _boundary->InBoundary(extendedCfg);
  if(reached) {
    if(this->m_debug)
      std::cout << "\tExtension reached goal boundary." << std::endl;
    return;
  }

  // Some extenders use a variable distance, so retry if we got part-way there.
  // Note that the original Cfg reference may have been invalidated by the graph
  // expanding!
  const double extendedDistance = dm->Distance(g->GetVertex(_newVID),
      extendedCfg);
  if(extendedDistance < distance) {
    if(this->m_debug)
      std::cout << "\tExtension made progress but did not reach goal, retrying."
                << std::endl;
    TryGoalExtension(extended, _boundary);
  }
  else if(this->m_debug)
    std::cout << "\tExtension did not make progress." << std::endl;
}


typename MedialAxisRRT::VID
MedialAxisRRT::
ExtendWithGoalExtender(const VID _nearVID, const Cfg& _target, LPOutput& _lp,
    const bool _requireNew) {
  auto stats = this->GetStatClass();
  const std::string id = this->GetNameAndLabel() + "::Extend";
  MethodTimer mt(stats, id);
  stats->IncStat(id);

  auto e = this->GetMPLibrary()->GetExtender(m_goalExtender);
  const Cfg& qNear = this->GetRoadmap()->GetVertex(_nearVID);
  Cfg qNew(this->GetTask()->GetRobot());

  const bool success = e->Extend(qNear, _target, qNew, _lp);
  if(this->m_debug)
    std::cout << "Extending from VID " << _nearVID
              << "\n\tqNear: " << qNear.PrettyPrint()
              << "\n\tExtended "
              << std::setprecision(4) << _lp.m_edge.first.GetWeight()
              << " units."
              << std::endl;

  if(!success) {
    // The extension failed to exceed the minimum distance.
    if(this->m_debug)
      std::cout << "\tNode too close, not adding." << std::endl;
    return INVALID_VID;
  }

  // The extension succeeded. Try to add the node.
  const auto extension = AddNode(qNew);

  const VID& newVID = extension.first;
  const bool nodeIsNew = extension.second;
  if(!nodeIsNew) {
    // The extension reproduced an existing node.
    if(_requireNew) {
      if(this->m_debug)
        std::cout << "\tNode already exists (" << newVID
                  << "), not adding." << std::endl;
      return INVALID_VID;
    }
    else if(this->m_debug)
      std::cout << "\tConnected to existing node " << newVID << "."
                << std::endl;
  }

  // The node was ok. Add the edge.
  AddEdge(_nearVID, newVID, _lp);

  return newVID;
}

typename MedialAxisRRT::VID
MedialAxisRRT::
ExtendWithGoalExtender(const VID _nearVID, const Cfg& _target, const bool _requireNew) {
  LPOutput dummyLP;
  return this->ExtendWithGoalExtender(_nearVID, _target, dummyLP, _requireNew);
}
