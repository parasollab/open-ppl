#ifndef PPL_GROUP_LOCAL_PLAN_H_
#define PPL_GROUP_LOCAL_PLAN_H_

#include "ConfigurationSpace/CompositeEdge.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/Weight.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include "containers/sequential/graph/graph_util.h"

#include <algorithm>
#include <iostream>
#include <vector>

template <typename GraphType> class GroupCfg;

////////////////////////////////////////////////////////////////////////////////
/// A local plan for multiple robots. Each robot will be executing this motion
/// simultaneously, and the total time is the same for each robot.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
class GroupLocalPlan : public CompositeEdge<GraphType> {

  public:

    ///@name Local Types
    ///@{

    typedef CompositeEdge<GraphType>                   BaseType;
    typedef typename BaseType::GroupGraphType          GroupGraphType;
    typedef typename GraphType::CfgType                CfgType;
    typedef typename GraphType::EdgeType               IndividualEdge;
    typedef double                                     EdgeWeight;

    typedef GroupCfg<GraphType>                        GroupCfgType;
    typedef GroupRoadmap<GroupCfgType, GroupLocalPlan> GroupRoadmapType;
    typedef std::vector<GroupCfgType>                  GroupCfgPath;
    typedef typename BaseType::CompositePath           CompositePath;
    typedef stapl::edge_descriptor_impl<size_t>        ED;

    /// A formation represents a group of robots which are maintaining their
    /// configurations relative to a leader, such as maintaining a square or
    /// V-shape while moving. The values are robot indexes (within the group,
    /// not problem) with the first index denoting the leader robot.
    typedef std::vector<size_t> Formation;

    ///@}
    ///@name Construction
    ///@{

    /// Constructs a GroupLocalPlan.
    /// @param _g The group roadmap in which this edge exists.
    /// @param _lpLabel The string label to assign to this plan. Defaults to empty string.
    /// @param _w The weight of the plan. Defaults to 0.0.
    /// @param _path The path to be given by the plan. Defaults to GroupCfgPath().
    GroupLocalPlan(GroupRoadmapType* const& _g = nullptr,
        const std::string& _lpLabel = "",
        const double _w = 0.0, const GroupCfgPath& _path = GroupCfgPath());

    /// Constructs a GroupLocalPlan.
    /// @param _g The robot group for which this edge exists.
    /// @param _lpLabel The string label to assign to this plan. Defaults to empty string.
    /// @param _w The weight of the plan. Defaults to 0.0.
    /// @param _path The path to be given by the plan. Defaults to GroupCfgPath().
    GroupLocalPlan(RobotGroup* const& _g,
        const std::string& _lpLabel = "",
        const double _w = 0.0, const GroupCfgPath& _path = GroupCfgPath());

    virtual ~GroupLocalPlan() = default;

    ///@}
    ///@name Misc. Interface Functions
    ///@{

    /// Set the group roadmap to which this plan belongs.
    /// @param _g The given group roadmap.
    void SetGroupRoadmap(GroupRoadmapType* const& _g);

    /// Set the formation of the robots. The first index is the leader.
    /// @param _indices The vector of robot indices given.
    void SetFormation(const Formation& _indices);

    /// Get the formation of the robots. The first index is the leader.
    /// @return The vector of formation robot indices.
    const Formation& GetFormation() const noexcept;
    
    /// Reset the states of this object.
    void Clear() noexcept;

    /// Get the group configuration intermediates.
    GroupCfgPath& GetIntermediates() noexcept;

    /// Get the group configuration intermediates.
    const GroupCfgPath& GetIntermediates() const noexcept;

    /// Set the group configuration intermediates.
    void SetIntermediates(const GroupCfgPath& _cfgs);

    /// Get the string label of this current local plan.
    const std::string& GetLPLabel() const noexcept;

    /// Set the string label of this current local plan.
    /// @param _label The desired string label.
    void SetLPLabel(const std::string _label) noexcept;

    ///@}
    ///@name Stapl graph interface
    ///@{

    /// Add two group local plans. This only adds weights, it doesn't take 
    /// intermediates into account.
    virtual GroupLocalPlan operator+(const GroupLocalPlan& _other) const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_lpLabel;   ///< Label of local planner that built this edge.

    /// The ordered formation for this local plan with respect to the robots
    /// in m_group. The first robot in the list is assumed to be the leader.
    std::vector<size_t> m_formation;

    /// The group cfg intermediates along this group local plan.
    GroupCfgPath m_cfgIntermediates;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename GraphType>
GroupLocalPlan<GraphType>::
GroupLocalPlan(GroupRoadmapType* const & _g, const std::string& _lpLabel,
    const double _w, const GroupCfgPath& _intermediates)
    : CompositeEdge<GraphType>((GroupGraphType*)_g, _w, (CompositePath&)_intermediates), 
    m_lpLabel(_lpLabel) {}

template <typename GraphType>
GroupLocalPlan<GraphType>::
GroupLocalPlan(RobotGroup* const & _g, const std::string& _lpLabel,
    const double _w, const GroupCfgPath& _intermediates)
    : CompositeEdge<GraphType>(_g, _w, (CompositePath&)_intermediates), 
    m_lpLabel(_lpLabel) {}

/*------------------------- Misc Interface Functions -------------------------*/

template <typename GraphType>
void
GroupLocalPlan<GraphType>::
SetGroupRoadmap(GroupRoadmapType* const& _g) {
  // The new composite graph must have the same group.
  if(_g->GetGroup() != this->m_group)
    throw RunTimeException(WHERE) << "The new group roadmap must have the "
                                  << "same robot group.";

  this->m_groupMap = (GroupGraphType*)_g;

  // Put all individual edges into the group local plan so that all are local:
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    this->SetEdge(i, IndividualEdge(this->GetEdge(i)));
}

template <typename GraphType>
void
GroupLocalPlan<GraphType>::
SetFormation(const Formation& _indices) {
  m_formation = _indices;
}


template <typename GraphType>
const typename GroupLocalPlan<GraphType>::Formation&
GroupLocalPlan<GraphType>::
GetFormation() const noexcept {
  return m_formation;
}


template <typename GraphType>
void
GroupLocalPlan<GraphType>::
Clear() noexcept {
  // Reset the initial state variables of this object:
  m_lpLabel.clear();
  this->m_weight = 0.;
  m_cfgIntermediates.clear();
  m_formation.clear();
}


template <typename GraphType>
typename GroupLocalPlan<GraphType>::GroupCfgPath&
GroupLocalPlan<GraphType>::
GetIntermediates() noexcept {
  return m_cfgIntermediates;
}


template <typename GraphType>
const typename GroupLocalPlan<GraphType>::GroupCfgPath&
GroupLocalPlan<GraphType>::
GetIntermediates() const noexcept {
  return m_cfgIntermediates;
}


template <typename GraphType>
void
GroupLocalPlan<GraphType>::
SetIntermediates(const GroupCfgPath& _cfgs) {
  m_cfgIntermediates = _cfgs;
}


template <typename GraphType>
const std::string&
GroupLocalPlan<GraphType>::
GetLPLabel() const noexcept {
  return m_lpLabel;
}


template <typename GraphType>
void
GroupLocalPlan<GraphType>::
SetLPLabel(const std::string _label) noexcept {
  m_lpLabel = _label;
}
 
/*---------------------- stapl graph interface helpers -----------------------*/

template <typename GraphType>
GroupLocalPlan<GraphType>
GroupLocalPlan<GraphType>::
operator+(const GroupLocalPlan& _other) const {
  if(!this->m_groupMap)
    return GroupLocalPlan(this->m_group, m_lpLabel,
                          this->m_weight + _other.m_weight);
  else
    return GroupLocalPlan((GroupRoadmapType*)this->m_groupMap, m_lpLabel,
                          this->m_weight + _other.m_weight);
}

/*------------------------------ Input/Output --------------------------------*/

template<typename GraphType>
std::ostream&
operator<<(std::ostream& _os, const GroupLocalPlan<GraphType>& _groupLP) {
  // For the group edges, the only caveat is that the intermediates need to line up.
  // Each individual edge within a GroupLocalPlan should have the same number of
  // intermediates (for now) so that we can do this easily. Then for a
  // GroupLocalPlan with individual edges for i robots, you would print all i
  // of the nth intermediates, then the (n+1)th, etc.

  // Make a vector of edges corresponding to each robot's edge to prevent from
  // repeatedly retrieving each vector of cfgs.
#if 0
  // TODO: when group intermediates are needed, use this code (but it's untested
  //       right now). Also it should really just populate m_intermediates and
  //       then print that vector (see DefaultWeight::Write() for analogous code).
  std::vector< std::vector<CfgType> > edgeIntermediates;
  const size_t numRobots = _groupLP.GetNumRobots();
  size_t numIntermediates = 0;
  const std::vector<size_t>& formation = _groupLP.GetFormation();
  for(size_t i = 0; i < numRobots; ++i) {
    // Check if the robot is inactive, if so, just duplicate the start cfg:
    if(std::find(formation.begin(), formation.end(), i) ==
                                                           formation.end()) {
      // Get the cfg that the robot is stationary at. Will be resized later to
      // account for correct number of intermediates.
      edgeIntermediates.push_back({_groupLP.GetRobotStartCfg(i)});
    }
    else {
      edgeIntermediates.push_back(_groupLP.GetEdge(i)->GetIntermediates());
      numIntermediates = edgeIntermediates.back().size();
    }
  }

  if(numIntermediates == 0)
    throw RunTimeException(WHERE, "No active robots were detected in an edge!");

  // Now all intermediate vectors of size 1 need to have their cfgs duplicated
  for(std::vector<CfgType>& intermediateVec : edgeIntermediates)
    if(intermediateVec.size() == 1)
      intermediateVec.resize(numIntermediates, intermediateVec[0]);
  // TODO: this could be optimized by not duplicating but won't be for now.

  // Now loop through all intermediates so we construct each intermediate's
  // composite cfg preserving the order of the robots in the group.
  // Note: assuming the same number of intermediates in each edge.
  for(size_t i = 0; i < numIntermediates; ++i) {
    for(size_t robot = 0; i < numRobots; ++i) {
      _os << edgeIntermediates[robot][i] << " ";
    }
  }

#endif

  _os << 0 << " "; // 0 intermediates for now.
  _os << _groupLP.Weight(); // Print out the weight.

  return _os;
}


#endif
