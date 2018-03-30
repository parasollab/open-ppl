#ifndef GROUP_LOCAL_PLAN_H_
#define GROUP_LOCAL_PLAN_H_

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/Weight.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include "containers/sequential/graph/graph_util.h"

#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// A local plan for multiple robots. Each robot will be executing this motion
/// simultaneously, and the total time is the same for each robot.
////////////////////////////////////////////////////////////////////////////////
template <typename CfgType>
class GroupLocalPlan final {

  public:

    ///@name Local Types
    ///@{

    typedef double                                         EdgeWeight;
    typedef DefaultWeight<CfgType>                         IndividualEdge;
    typedef GroupRoadmap<GroupCfg, GroupLocalPlan>          GraphType;

    typedef stapl::edge_descriptor_impl<size_t> ED;

    ///@}
    ///@name Construction
    ///@{

    GroupLocalPlan(GraphType* const _g = nullptr);

    ///@}
    ///@name Ordering and Equality
    ///@{

    virtual bool operator==(const GroupLocalPlan& _w) const noexcept;
    virtual bool operator!=(const GroupLocalPlan& _w) const noexcept;

    virtual bool operator<(const GroupLocalPlan& _other) const noexcept;

    ///@}
    ///@name Roadmap Edge Weight
    ///@{
    /// Get/set the numeric weight for this local plan as used when querying a
    /// roadmap.

    EdgeWeight GetWeight() const noexcept;

    void SetWeight(const EdgeWeight _w) noexcept;

    ///@}
    ///@name Individual Local Plans
    ///@{

    /// Set the individual edge for a robot to a local copy of an edge.
    /// @param _robot The robot which the edge refers to.
    /// @param _edge The edge.
    void SetEdge(Robot* const _robot, IndividualEdge&& _edge);

    /// Set the individual edge for a robot to a roadmap copy of an edge.
    /// @param _robot The robot which the edge refers to.
    /// @param _ed The edge descriptor.
    void SetEdge(Robot* const _robot, const ED _ed);

    /// Get the individual edge for a robot.
    /// @param _robot The robot which the edge refers to.
    /// @return A pointer to the edge for _robot. It will be null if it has not
    ///         yet been set.
    IndividualEdge* GetEdge(Robot* const _robot);

    /// Get the individual edge for a robot.
    /// @param _robot The robot which the edge refers to.
    /// @return A pointer to the edge for _robot. It will be null if it has not
    ///         yet been set.
    const IndividualEdge* GetEdge(Robot* const _robot) const;

    std::vector<IndividualEdge>& GetLocalEdges() noexcept;
    std::vector<ED>& GetEdgeDescriptors() noexcept;

    void ClearLocalEdges() noexcept;

    ///@}
    ///@name Iteration
    ///@{
    /// Iterate over the EDs in this edge.

    typedef typename std::vector<ED>::iterator       iterator;
    typedef typename std::vector<ED>::const_iterator const_iterator;

    iterator begin() noexcept;
    iterator end() noexcept;

    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;

    ///@}

  private:

    ///@name Internal State
    ///@{

    GraphType* m_groupMap{nullptr};  ///< The robot group which follows this edge.

    /// Note that any edges added to m_localEdges must be complete.
    std::vector<IndividualEdge> m_localEdges; ///< Edges which are not in a map.
    std::vector<ED> m_edges;                  ///< Descriptors of the individual edges.

    /// The edge weight.
    double m_weight{std::numeric_limits<double>::infinity()};

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename CfgType>
GroupLocalPlan<CfgType>::
GroupLocalPlan(GraphType* const _g) : m_groupMap(_g),
    m_edges(m_groupMap->GetGroup()->Size(), INVALID_ED) {
}

/*--------------------------- Ordering and Equality --------------------------*/

template <typename CfgType>
bool
GroupLocalPlan<CfgType>::
operator==(const GroupLocalPlan& _other) const noexcept {
  // Ensure the edges belong to the same group.
  if(m_groupMap->GetGroup() != _other.m_groupMap->GetGroup())
    return false;

  // Ensure the edges are equal.
  for(size_t i = 0; i < m_groupMap->GetGroup()->Size(); ++i) {
    // If both descriptors are valid and equal, these edges are equal.
    const auto& ed1 = m_edges[i],
              & ed2 = _other.m_edges[i];
    if(ed1 != INVALID_ED and ed2 != INVALID_ED and ed1 != ed2)
      return false;

    // If either descriptor is invalid, one of the edges must be a local edge.
    // In that case, these edges must compare value-equal.
    const auto robot1 = m_groupMap->GetGroup()->GetRobot(i),
               robot2 = _other.m_groupMap->GetGroup()->GetRobot(i);
    if(*GetEdge(robot1) != *_other.GetEdge(robot2))
      return false;
  }

  return true;
}


template <typename CfgType>
bool
GroupLocalPlan<CfgType>::
operator!=(const GroupLocalPlan& _other) const noexcept {
  return !(*this == _other);
}


template <typename CfgType>
inline
bool
GroupLocalPlan<CfgType>::
operator<(const GroupLocalPlan& _other) const noexcept {
  return GetWeight() < _other.GetWeight();
}

/*---------------------------- Roadmap Edge Weights --------------------------*/

template <typename CfgType>
typename GroupLocalPlan<CfgType>::EdgeWeight
GroupLocalPlan<CfgType>::
GetWeight() const noexcept {
  return m_weight;
}


template <typename CfgType>
void
GroupLocalPlan<CfgType>::
SetWeight(const EdgeWeight _w) noexcept {
  m_weight = _w;
}

/*-------------------------- Individual Local Plans --------------------------*/

template <typename CfgType>
void
GroupLocalPlan<CfgType>::
SetEdge(Robot* const _robot, IndividualEdge&& _edge) {
  // Allocate space for local edges if not already done.
  m_localEdges.resize(m_groupMap->GetGroup()->Size());

  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);

  m_localEdges[index] = std::move(_edge);
  m_edges[index] = INVALID_ED;
}


template <typename CfgType>
void
GroupLocalPlan<CfgType>::
SetEdge(Robot* const _robot, const ED _ed) {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);

  m_edges[index] = _ed;
}


template <typename CfgType>
typename GroupLocalPlan<CfgType>::IndividualEdge*
GroupLocalPlan<CfgType>::
GetEdge(Robot* const _robot) {
  return const_cast<IndividualEdge*>(GetEdge(_robot));
}


template <typename CfgType>
const typename GroupLocalPlan<CfgType>::IndividualEdge*
GroupLocalPlan<CfgType>::
GetEdge(Robot* const _robot) const {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);

  const ED& descriptor = m_edges.at(index);
  if(descriptor == INVALID_ED) {
    try {
      return &m_localEdges.at(index);
    }
    catch(const std::runtime_error&) {
      std::ostringstream oss;
      oss << "Requested edge (" << descriptor.source() << ", "
          << descriptor.target() << ") for robot " << index
          << " (" << _robot << "), which is not in the group map.";
      throw RunTimeException(WHERE, oss.str());
    }
  }

  return &m_groupMap->GetRoadmap(index)->GetEdge(descriptor.source(),
      descriptor.target());
}


template <typename CfgType>
std::vector<typename GroupLocalPlan<CfgType>::IndividualEdge>&
GroupLocalPlan<CfgType>::
GetLocalEdges() noexcept {
  return m_localEdges;
}


template <typename CfgType>
std::vector<typename GroupLocalPlan<CfgType>::ED>&
GroupLocalPlan<CfgType>::
GetEdgeDescriptors() noexcept {
  return m_edges;
}


template <typename CfgType>
void
GroupLocalPlan<CfgType>::
ClearLocalEdges() noexcept {
  m_localEdges.clear();
}

/*-------------------------------- Iteration ---------------------------------*/

template <typename CfgType>
typename GroupLocalPlan<CfgType>::iterator
GroupLocalPlan<CfgType>::
begin() noexcept {
  return m_edges.begin();
}


template <typename CfgType>
typename GroupLocalPlan<CfgType>::iterator
GroupLocalPlan<CfgType>::
end() noexcept {
  return m_edges.end();
}


template <typename CfgType>
typename GroupLocalPlan<CfgType>::const_iterator
GroupLocalPlan<CfgType>::
begin() const noexcept {
  return m_edges.begin();
}


template <typename CfgType>
typename GroupLocalPlan<CfgType>::const_iterator
GroupLocalPlan<CfgType>::
end() const noexcept {
  return m_edges.end();
}

/*----------------------------------------------------------------------------*/

#endif
