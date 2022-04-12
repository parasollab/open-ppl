#ifndef PPL_COMPOSITE_EDGE_H_
#define PPL_COMPOSITE_EDGE_H_

#include "ConfigurationSpace/Weight.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "ConfigurationSpace/CompositeGraph.h"

#include "containers/sequential/graph/graph_util.h"

#include <algorithm>
#include <iostream>
#include <vector>


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
class CompositeEdge {

  public:

    ///@name Local Types
    ///@{

    typedef typename GraphType::Vertex CfgType;
    typedef typename GraphType::Edge   IndividualEdge;

    typedef double                                  EdgeWeight;
    // typedef DefaultWeight<GraphType>                  IndividualEdge;
    typedef CompositeGraph<GraphType>  GroupRoadmapType;
    // typedef std::vector<GroupCfg>                   GroupCfgPath;
    // typedef size_t                                  GroupVID;

    typedef stapl::edge_descriptor_impl<size_t>     ED;

    ///@}
    ///@name Construction
    ///@{

    /// Constructs a CompositeEdge.
    /// @param _g The group of robots to follow this edge. Defaults to nullptr.
    /// @param _lpLabel The string label to assign to this plan. Defaults to empty string.
    /// @param _w The weight of the plan. Defaults to 0.0.
    /// @param _path The path to be given by the plan. Defaults to GroupCfgPath().
    CompositeEdge(GroupRoadmapType* const _g = nullptr, const double _w = 0.0);

    ///@}
    ///@name Ordering and Equality
    ///@{

    /// Check if the given plan is equal to the current.
    /// @param _w The given plan.
    virtual bool operator==(const CompositeEdge& _w) const noexcept;

    /// Check if the given plan is unequal to the current.
    /// @param _w The given plan.
    virtual bool operator!=(const CompositeEdge& _w) const noexcept;

    /// Check if the given plan is less than the current.
    /// @param _other The given plan.
    virtual bool operator<(const CompositeEdge& _other) const noexcept;

    ///@}
    ///@name Roadmap Edge Weight
    ///@{
    /// Get/set the numeric weight for this local plan as used when querying a
    /// roadmap.

    EdgeWeight GetWeight() const noexcept;

    void SetWeight(const EdgeWeight _w) noexcept;

    ///@}
    ///@name Misc. Interface Functions
    ///@{

    // There is no current use case where these should ever get reset to false.
    void SetSkipEdge() noexcept;
    bool SkipEdge() const noexcept;
    
    /// Reset the states of this object.
    void Clear() noexcept;

    ///@}
    ///@name Individual Local Plans
    ///@{

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

    ///Overloads for using index instead of robot pointer.
    IndividualEdge* GetEdge(const size_t _robotIndex);
    const IndividualEdge* GetEdge(const size_t _robotIndex) const;

    /// Get a vector of local edges in the plan.
    std::vector<IndividualEdge>& GetLocalEdges() noexcept;

    /// Get a vector of local edges' descriptors.
    std::vector<ED>& GetEdgeDescriptors() noexcept;

    /// Get the number of robots given in this group local plan.
    size_t GetNumRobots() const noexcept;

    void SetTimeSteps(size_t _timesteps);

    size_t GetTimeSteps() const;

    ///@}
    ///@name Stapl graph interface
    ///@{

    /// This only adds weights, it doesn't take intermediates into account.
    virtual CompositeEdge operator+(const CompositeEdge& _other) const ;

    /// Get the weight of the plan.
    double Weight() const noexcept;

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

    GroupRoadmapType* m_groupMap{nullptr};  ///< The robot group which follows this edge.

    /// The edge weight.
    double m_weight{std::numeric_limits<double>::infinity()};

    std::vector<ED> m_edges;           ///< Descriptors of the individual edges.

    bool m_skipEdge{false}; ///< Flag to skip full recreation in GroupPath::FullCfgs.

    size_t m_timesteps;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename GraphType>
CompositeEdge<GraphType>::
CompositeEdge(GroupRoadmapType* const _g, const double _w)
    : m_groupMap(_g), m_weight(_w) {
  if(m_groupMap)
    m_edges.resize(m_groupMap->GetGroup()->Size(), INVALID_ED);
  //else
    //std::cout << "Warning: no group map provided in group LP!" << std::endl;
}

/*--------------------------- Ordering and Equality --------------------------*/

template <typename GraphType>
bool
CompositeEdge<GraphType>::
operator==(const CompositeEdge& _other) const noexcept {
   
  // Check that both edges have a group map
  // If neither do, return true
  if(!m_groupMap and !_other.m_groupMap)
    return true;

  // If only one does, return false
  if(!m_groupMap or !_other.m_groupMap)
    return false;
 
  // Ensure the edges belong to the same group.
  if(m_groupMap->GetGroup() != _other.m_groupMap->GetGroup())
    return false;

  // Ensure the edges are equal.
//  for(size_t i = 0; i < m_groupMap->GetGroup()->Size(); ++i) {
  for(const size_t i : m_groupMap->GetGroup()->Size()) {
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


template <typename GraphType>
bool
CompositeEdge<GraphType>::
operator!=(const CompositeEdge& _other) const noexcept {
  return !(*this == _other);
}


template <typename GraphType>
inline
bool
CompositeEdge<GraphType>::
operator<(const CompositeEdge& _other) const noexcept {
  return GetWeight() < _other.GetWeight();
}

/*---------------------------- Roadmap Edge Weights --------------------------*/

template <typename GraphType>
typename CompositeEdge<GraphType>::EdgeWeight
CompositeEdge<GraphType>::
GetWeight() const noexcept {
  return m_weight;
}


template <typename GraphType>
void
CompositeEdge<GraphType>::
SetWeight(const EdgeWeight _w) noexcept {
  m_weight = _w;
}

/*------------------------- Misc Interface Functions -------------------------*/

template <typename GraphType>
void
CompositeEdge<GraphType>::
SetSkipEdge() noexcept {
  m_skipEdge = true;
}


template <typename GraphType>
bool
CompositeEdge<GraphType>::
SkipEdge() const noexcept {
  return m_skipEdge;
}


template <typename GraphType>
void
CompositeEdge<GraphType>::
Clear() noexcept {
  // Reset the initial state variables of this object:
  m_weight = 0.;
}

/*-------------------------- Individual Local Plans --------------------------*/

template <typename GraphType>
void
CompositeEdge<GraphType>::
SetEdge(Robot* const _robot, const ED _ed) {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);

  m_edges[index] = _ed;
}


template <typename GraphType>
typename CompositeEdge<GraphType>::IndividualEdge*
CompositeEdge<GraphType>::
GetEdge(const size_t _robotIndex) {
  return const_cast<IndividualEdge*>(GetEdge(
                          this->m_groupMap->GetGroup()->GetRobot(_robotIndex)));
}


template <typename GraphType>
const typename CompositeEdge<GraphType>::IndividualEdge*
CompositeEdge<GraphType>::
GetEdge(const size_t _robotIndex) const {
  return GetEdge(this->m_groupMap->GetGroup()->GetRobot(_robotIndex));
}


template <typename GraphType>
typename CompositeEdge<GraphType>::IndividualEdge*
CompositeEdge<GraphType>::
GetEdge(Robot* const _robot) {
  return const_cast<IndividualEdge*>(GetEdge(_robot));
}


template <typename GraphType>
const typename CompositeEdge<GraphType>::IndividualEdge*
CompositeEdge<GraphType>::
GetEdge(Robot* const _robot) const {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);

  const ED& descriptor = m_edges.at(index);
  if(descriptor != INVALID_ED)
    return &m_groupMap->GetRoadmap(index)->GetEdge(descriptor.source(),
                                                   descriptor.target());

  throw RunTimeException(WHERE) << "Requested individual edge for robot "
                                << index << " (" << _robot << "), which is"
                                << " either stationary for this LP or not "
                                << "in the group.";
}


template <typename GraphType>
std::vector<typename CompositeEdge<GraphType>::ED>&
CompositeEdge<GraphType>::
GetEdgeDescriptors() noexcept {
  return m_edges;
}


template <typename GraphType>
size_t
CompositeEdge<GraphType>::
GetNumRobots() const noexcept {
  return m_groupMap->GetGroup()->Size();
}
    
template <typename GraphType>
void 
CompositeEdge<GraphType>::
SetTimeSteps(size_t _timesteps) {
  m_timesteps = _timesteps;
}

template <typename GraphType>
size_t 
CompositeEdge<GraphType>::
GetTimeSteps() const {
  return m_timesteps;
}
 
/*---------------------- stapl graph interface helpers -----------------------*/

template <typename GraphType>
CompositeEdge<GraphType>
CompositeEdge<GraphType>::
operator+(const CompositeEdge& _other) const {
  return CompositeEdge(m_groupMap, m_weight + _other.m_weight);
}


template <typename GraphType>
double
CompositeEdge<GraphType>::
Weight() const noexcept {
  return GetWeight();
}

/*-------------------------------- Iteration ---------------------------------*/

template <typename GraphType>
typename CompositeEdge<GraphType>::iterator
CompositeEdge<GraphType>::
begin() noexcept {
  return m_edges.begin();
}


template <typename GraphType>
typename CompositeEdge<GraphType>::iterator
CompositeEdge<GraphType>::
end() noexcept {
  return m_edges.end();
}


template <typename GraphType>
typename CompositeEdge<GraphType>::const_iterator
CompositeEdge<GraphType>::
begin() const noexcept {
  return m_edges.begin();
}


template <typename GraphType>
typename CompositeEdge<GraphType>::const_iterator
CompositeEdge<GraphType>::
end() const noexcept {
  return m_edges.end();
}

#endif
