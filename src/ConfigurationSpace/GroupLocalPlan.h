#ifndef PPL_GROUP_LOCAL_PLAN_H_
#define PPL_GROUP_LOCAL_PLAN_H_

#include "ConfigurationSpace/CompositeEdge.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/CompositeGraph.h"
#include "ConfigurationSpace/GroupCfg.h"
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
///
/// @todo Remove the 'active robots'. All robots are always active in a group
///       edge. We do need to retain formation data for reconstructing edges.
/// @todo Remove the 'skip edge' stuff. There is no such thing as a skipped
///       edge: the disassembly code needs to be adjusted to only use the edges
///       they want.
/// @todo Rework so that we only need a robot group to construct this, in which
///       case it will have all local edges. It should only get tied to a
///       roadmap with SetGroupRoadmap after it has been added to one.
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
    typedef CompositeGraph<GroupCfgType, GroupLocalPlan> GroupRoadmapType;
    typedef std::vector<GroupCfgType>                  GroupCfgPath;
    typedef typename BaseType::CompositePath           CompositePath;
    typedef stapl::edge_descriptor_impl<size_t>        ED;

    ///@}
    ///@name Construction
    ///@{

    /// Constructs a GroupLocalPlan.
    /// @param _g The group roadmap in which this edge exists. Defaults to nullptr.
    /// @param _lpLabel The string label to assign to this plan. Defaults to empty string.
    /// @param _w The weight of the plan. Defaults to 0.0.
    /// @param _path The path to be given by the plan. Defaults to GroupCfgPath().
    GroupLocalPlan(GroupRoadmapType* const& _g = nullptr,
        const std::string& _lpLabel = "",
        const double _w = 0.0, const GroupCfgPath& _path = GroupCfgPath());

    virtual ~GroupLocalPlan() = default;

    ///@}
    ///@name Misc. Interface Functions
    ///@{

    /// Set the list of active robots to the vector of robot indices given.
    /// @param The vector of robo indices given.
    void SetActiveRobots(const std::vector<size_t>& _indices);
    /// Get the vector of active robots indices.
    /// @return The vector of active robot indices.
    const std::vector<size_t>& GetActiveRobots() const noexcept;
    
    /// Reset the states of this object.
    void Clear() noexcept;

    using CompositeEdge<GraphType>::GetIntermediates;

    /// Get the group configuration intermediates.
    GroupCfgPath& GetIntermediates() noexcept;
    /// Get the group configuration intermediates.
    const GroupCfgPath& GetIntermediates() const noexcept;

    using CompositeEdge<GraphType>::SetIntermediates;

    /// Set the group configuration intermediates.
    void SetIntermediates(const GroupCfgPath& _cfgs);

    /// Get the string label of this current local plan.
    const std::string& GetLPLabel() const noexcept;
    /// Set the string label of this current local plan.
    /// @param The desired string label.
    void SetLPLabel(const std::string _label) noexcept;

    ///@}
    ///@name Individual Local Plans
    ///@{

    using CompositeEdge<GraphType>::SetEdge;

    /// Set the individual edge for a robot to a local copy of an edge.
    /// @param _robot The robot which the edge refers to.
    /// @param _edge The edge.
    void SetEdge(const size_t _robot, IndividualEdge&& _edge);

    /// overload for Robot pointer
    void SetEdge(Robot* const _robot, IndividualEdge&& _edge);

    using CompositeEdge<GraphType>::GetEdge;

    /// Get the individual edge for a robot.
    /// @param _robot The robot which the edge refers to.
    /// @return A pointer to the edge for _robot. It will be null if it has not
    ///         yet been set.
    const IndividualEdge* GetEdge(Robot* const _robot) const override;

    /// Get a vector of local edges in the plan.
    std::vector<IndividualEdge>& GetLocalEdges() noexcept;

    /// Clear all local edges in the plan.
    void ClearLocalEdges() noexcept;

    ///@}
    ///@name Stapl graph interface
    ///@{

    /// This only adds weights, it doesn't take intermediates into account.
    virtual GroupLocalPlan operator+(const GroupLocalPlan& _other) const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_lpLabel;   ///< Label of local planner that built this edge.

    // The ordered formation for this local plan with respect to the robots
    // in m_groupMap. The first robot in the list is assumed to be the leader.
    std::vector<size_t> m_activeRobots;

    /// Note that any edges added to m_localEdges must be valid and complete.
    std::vector<IndividualEdge> m_localEdges; ///< Edges which are not in a map.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename GraphType>
GroupLocalPlan<GraphType>::
GroupLocalPlan(GroupRoadmapType* const & _g, const std::string& _lpLabel,
    const double _w, const GroupCfgPath& _intermediates)
    : CompositeEdge<GraphType>((GroupGraphType*)_g, _w, (CompositePath&)_intermediates), 
    m_lpLabel(_lpLabel) {}

/*------------------------- Misc Interface Functions -------------------------*/

template <typename GraphType>
void
GroupLocalPlan<GraphType>::
SetActiveRobots(const std::vector<size_t>& _indices) {
  m_activeRobots = _indices;
}


template <typename GraphType>
const std::vector<size_t>&
GroupLocalPlan<GraphType>::
GetActiveRobots() const noexcept {
  return m_activeRobots;
}


template <typename GraphType>
void
GroupLocalPlan<GraphType>::
Clear() noexcept {
  // Reset the initial state variables of this object:
  m_lpLabel.clear();
  this->m_weight = 0.;
  this->m_intermediates.clear();
}


template <typename GraphType>
typename GroupLocalPlan<GraphType>::GroupCfgPath&
GroupLocalPlan<GraphType>::
GetIntermediates() noexcept {
  return (GroupCfgPath&)this->GetIntermediates();
}


template <typename GraphType>
const typename GroupLocalPlan<GraphType>::GroupCfgPath&
GroupLocalPlan<GraphType>::
GetIntermediates() const noexcept {
  return (GroupCfgPath&)this->GetIntermediates();
}


template <typename GraphType>
void
GroupLocalPlan<GraphType>::
SetIntermediates(const GroupCfgPath& _cfgs) {
  this->SetIntermediates((CompositePath&)_cfgs);
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

/*-------------------------- Individual Local Plans --------------------------*/

template <typename GraphType>
void
GroupLocalPlan<GraphType>::
SetEdge(Robot* const _robot, IndividualEdge&& _edge) {
  const size_t index = this->m_groupMap->GetGroup()->GetGroupIndex(_robot);
  SetEdge(index, std::move(_edge));
}


template <typename GraphType>
void
GroupLocalPlan<GraphType>::
SetEdge(const size_t robotIndex, IndividualEdge&& _edge) {
  // Allocate space for local edges if not already done.
  m_localEdges.resize(this->m_groupMap->GetGroup()->Size());

  m_localEdges[robotIndex] = std::move(_edge);
  this->m_edges[robotIndex] = INVALID_ED;
}


template <typename GraphType>
const typename GroupLocalPlan<GraphType>::IndividualEdge*
GroupLocalPlan<GraphType>::
GetEdge(Robot* const _robot) const {
  const size_t index = this->m_groupMap->GetGroup()->GetGroupIndex(_robot);

  const ED& descriptor = this->m_edges.at(index);
  if(descriptor != INVALID_ED)
    return &this->m_groupMap->GetRoadmap(index)->GetEdge(descriptor.source(),
                                                   descriptor.target());

  try {
    return &m_localEdges.at(index);
  }
  catch(const std::out_of_range&) {
    throw RunTimeException(WHERE) << "Requested individual edge for robot "
                                  << index << " (" << _robot << "), which is"
                                  << " either stationary for this LP or not "
                                  << "in the group.";
  }
}


template <typename GraphType>
std::vector<typename GroupLocalPlan<GraphType>::IndividualEdge>&
GroupLocalPlan<GraphType>::
GetLocalEdges() noexcept {
  return m_localEdges;
}


template <typename GraphType>
void
GroupLocalPlan<GraphType>::
ClearLocalEdges() noexcept {
  m_localEdges.clear();
}
 
/*---------------------- stapl graph interface helpers -----------------------*/

template <typename GraphType>
GroupLocalPlan<GraphType>
GroupLocalPlan<GraphType>::
operator+(const GroupLocalPlan& _other) const {
  return GroupLocalPlan((GroupRoadmapType*)this->m_groupMap, m_lpLabel,
                        this->m_weight + _other.m_weight);
}

/*------------------------------ Input/Output --------------------------------*/

template<typename GraphType>
std::ostream&
operator<<(std::ostream& _os, const GroupLocalPlan<GraphType>& _groupLP) {
  //For the group edges, the only caveat is that the intermediates need to line up.
  // Each individual edge within a GroupLocalPlan should have the same number of
  // intermediates (for now) so that we can do this easily. Then for a
  // GroupLocalPlan with n individual edges for i robots, you would print all i
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
  const std::vector<size_t>& activeRobots = _groupLP.GetActiveRobots();
  for(size_t i = 0; i < numRobots; ++i) {
    // Check if the robot is inactive, if so, just duplicate the start cfg:
    if(std::find(activeRobots.begin(), activeRobots.end(), i) ==
                                                           activeRobots.end()) {
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
