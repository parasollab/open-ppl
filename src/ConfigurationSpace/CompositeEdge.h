#ifndef PPL_COMPOSITE_EDGE_H_
#define PPL_COMPOSITE_EDGE_H_

#include "ConfigurationSpace/Weight.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "ConfigurationSpace/CompositeState.h"
#include "ConfigurationSpace/CompositeGraph.h"

#include "containers/sequential/graph/graph_util.h"

#include <algorithm>
#include <iostream>
#include <vector>

template <typename GraphType> class CompositeState;


////////////////////////////////////////////////////////////////////////////////
/// A composite edge for multiple robots, which is composed of an individual
/// edge for each robot. 'GraphType' refers to the individual robot graph type.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
class CompositeEdge {

  public:

    ///@name Local Types
    ///@{

    typedef typename GraphType::CfgType    IndividualCfg;
    typedef typename GraphType::EdgeType   IndividualEdge;

    typedef double                               EdgeWeight;
    typedef stapl::edge_descriptor_impl<size_t>  ED;

    typedef CompositeState<GraphType>                         CompositeStateType;
    typedef CompositeGraph<CompositeStateType, CompositeEdge> GroupGraphType;
    typedef std::vector<CompositeStateType>                   CompositePath;

    ///@}
    ///@name Construction
    ///@{

    /// Constructs a CompositeEdge.
    /// @param _g The group roadmap in which this edge exists.
    /// @param _w The weight of the plan. Defaults to 0.0.
    /// @param _intermediates The intermediates along the edge. Defaults to CompositePath().
    CompositeEdge(GroupGraphType* const & _g = nullptr, const double _w = 0.0, 
      const CompositePath& _intermediates = CompositePath());

    /// Constructs a CompositeEdge.
    /// @param _g The robot group to which this edge exists.
    /// @param _w The weight of the plan. Defaults to 0.0.
    /// @param _intermediates The intermediates along the edge. Defaults to CompositePath().
    CompositeEdge(RobotGroup* const & _g, const double _w = 0.0, 
      const CompositePath& _intermediates = CompositePath());

    ///@}
    ///@name Ordering and Equality
    ///@{

    /// Check if the given edge is equal to the current.
    /// @param _w The given edge.
    virtual bool operator==(const CompositeEdge& _w) const noexcept;

    /// Check if the given edge is unequal to the current.
    /// @param _w The given edge.
    virtual bool operator!=(const CompositeEdge& _w) const noexcept;

    /// Check if the given edge is less than the current.
    /// @param _other The given edge.
    virtual bool operator<(const CompositeEdge& _other) const noexcept;

    ///@}
    ///@name Graph Edge Weight
    ///@{

    /// Get the numeric weight for this edge.
    virtual EdgeWeight GetWeight() const noexcept;

    /// Set the numeric weight for this edge.
    /// @param _w The numeric weight.
    virtual void SetWeight(const EdgeWeight _w) noexcept;

    ///@}
    ///@name Misc. Interface Functions
    ///@{

    /// Set the composite graph that this composite edge lies within.
    /// @param _g The composite graph to associate the edge with.
    void SetGroupGraph(GroupGraphType* const& _g);
    
    /// Reset the states of this object.
    virtual void Clear() noexcept;

    /// Get the composite state intermediates.
    CompositePath& GetIntermediates() noexcept;

    /// Get the composite state intermediates.
    const CompositePath& GetIntermediates() const noexcept;

    /// Get the number of composite intermediates along this edge.
    const size_t GetNumIntermediates() noexcept;

    /// Set the composite state intermediates.
    void SetIntermediates(const CompositePath& _cfgs);

    /// Write an edge to an output stream.
    /// @param _os The output stream to write to.
    virtual void Write(std::ostream& _os) const;

    ///@}
    ///@name Individual Local Plans
    ///@{

    /// Set the individual edge for a robot to a graph copy of an edge.
    /// @param _robot The robot which the edge refers to.
    /// @param _ed The edge descriptor.
    virtual void SetEdge(Robot* const _robot, const ED _ed);

    /// Set the individual edge for a robot to a local copy of an edge.
    /// @param _robot The robot which the edge refers to.
    /// @param _edge The edge.
    void SetEdge(const size_t _robot, IndividualEdge&& _edge);

    /// Set the individual edge for a robot to a local copy of an edge.
    /// @param _robot The robot which the edge refers to.
    /// @param _edge The edge.
    void SetEdge(Robot* const _robot, IndividualEdge&& _edge);

    /// Get the individual edge for a robot.
    /// @param _robot The robot which the edge refers to.
    /// @return A pointer to the edge for _robot. It will be null if it has not
    ///         yet been set.
    virtual IndividualEdge* GetEdge(Robot* const _robot);

    /// Get the individual edge for a robot.
    /// @param _robot The robot which the edge refers to.
    /// @return A pointer to the edge for _robot. It will be null if it has not
    ///         yet been set.
    virtual const IndividualEdge* GetEdge(Robot* const _robot) const;

    /// Get the individual edge for a robot.
    /// @param _robotIndex The group index of the robot which the edge refers to.
    /// @return A pointer to the edge for _robot. It will be null if it has not
    ///         yet been set.
    virtual IndividualEdge* GetEdge(const size_t _robotIndex);

    /// Get the individual edge for a robot.
    /// @param _robotIndex The group index of the robot which the edge refers to.
    /// @return A pointer to the edge for _robot. It will be null if it has not
    ///         yet been set.
    virtual const IndividualEdge* GetEdge(const size_t _robotIndex) const;

    /// Get a vector of local edges in the plan.
    std::vector<IndividualEdge>& GetLocalEdges() noexcept;

    /// Clear all local edges in the plan.
    void ClearLocalEdges() noexcept;

    /// Get a vector of individual edge descriptors. An edge descriptor will
    /// be invalid if an individual edge is local.
    virtual std::vector<ED>& GetEdgeDescriptors() noexcept;

    /// Get the number of robots given in this group local plan.
    virtual size_t GetNumRobots() const noexcept;

    /// Get a vector of the robots who move along this composite edge
    /// (i.e. the individual source vertex differs from the target vertex).
    virtual std::unordered_set<Robot*> GetActiveRobots();

    /// Set the number of timesteps along this composite edge.
    virtual void SetTimeSteps(size_t _timesteps);

    /// Get the number of timesteps along this composite edge.
    virtual size_t GetTimeSteps() const;

    ///@}
    ///@name Stapl graph interface
    ///@{

    /// Add two composite edges. This only adds weights, it doesn't take 
    /// intermediates into account.
    virtual CompositeEdge operator+(const CompositeEdge& _other) const ;

    /// Get the weight of the plan.
    virtual double Weight() const noexcept;

    ///@}
    ///@name Iteration
    ///@{
    /// Iterate over the EDs in this edge.

    typedef typename std::vector<ED>::iterator       iterator;
    typedef typename std::vector<ED>::const_iterator const_iterator;

    virtual iterator begin() noexcept;
    virtual iterator end() noexcept;

    virtual const_iterator begin() const noexcept;
    virtual const_iterator end() const noexcept;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    GroupGraphType* m_groupMap{nullptr}; ///< The composite graph that this edge is in.

    RobotGroup* m_group{nullptr}; ///< The robot group that this edge belongs to.

    /// The edge weight.
    double m_weight{std::numeric_limits<double>::infinity()};

    CompositePath m_intermediates; ///< Composite state intermediates.

    std::vector<ED> m_edges; ///< Descriptors of the individual edges.

    /// Note that any edges added to m_localEdges must be valid and complete.
    std::vector<IndividualEdge> m_localEdges; ///< Edges which are not in a map.

    size_t m_timesteps; ///< The number of timesteps along this edge.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename GraphType>
CompositeEdge<GraphType>::
CompositeEdge(GroupGraphType* const & _g, const double _w, 
    const CompositePath& _intermediates)
    : m_groupMap(_g), m_group(_g->GetGroup()), 
      m_weight(_w) {

  m_intermediates.clear();
  for(auto intermediate : _intermediates) {
    m_intermediates.push_back(intermediate);
  }

  if(m_groupMap) {
    m_edges.resize(m_groupMap->GetGroup()->Size(), INVALID_ED);
    m_localEdges.resize(m_groupMap->GetGroup()->Size());
  }
}

template <typename GraphType>
CompositeEdge<GraphType>::
CompositeEdge(RobotGroup* const & _g, const double _w, 
    const CompositePath& _intermediates)
    : m_group(_g), m_weight(_w), m_intermediates(_intermediates) {

  // Allocate space for local edges and set all edge descriptors to invalid.
  m_localEdges.resize(m_group->Size());
  m_edges.resize(m_group->Size(), INVALID_ED);
}

/*------------------------- Misc Interface Functions -------------------------*/

template <typename GraphType>
void
CompositeEdge<GraphType>::
SetGroupGraph(GroupGraphType* const & _g) {
  // The new composite graph must have the same group.
  if(_g->GetGroup() != m_group)
    throw RunTimeException(WHERE) << "The new composite graph must have the "
                                  << "same robot group.";

  // Clear the edge descriptors since they do not correspond to the new graph.
  m_edges.resize(m_group->Size(), INVALID_ED);

  m_groupMap = _g;
}

template <typename GraphType>
typename CompositeEdge<GraphType>::CompositePath&
CompositeEdge<GraphType>::
GetIntermediates() noexcept {
  return m_intermediates;
}


template <typename GraphType>
const typename CompositeEdge<GraphType>::CompositePath&
CompositeEdge<GraphType>::
GetIntermediates() const noexcept {
  return m_intermediates;
}


template <typename GraphType>
const size_t
CompositeEdge<GraphType>::
GetNumIntermediates() noexcept {
  return m_intermediates.size();
}


template <typename GraphType>
void
CompositeEdge<GraphType>::
SetIntermediates(const CompositePath& _path) {
  m_intermediates = _path;
}

/*--------------------------- Ordering and Equality --------------------------*/

template <typename GraphType>
bool
CompositeEdge<GraphType>::
operator==(const CompositeEdge& _other) const noexcept {

  // Check that both edges correspond to the same group.
  // If not, return false.
  if(m_group != _other.m_group)
    return false;
   
  // Check that both edges have a group map
  // If neither do, return true
  if(!m_groupMap and !_other.m_groupMap)
    return true;

  // If only one does, return false
  if(!m_groupMap or !_other.m_groupMap)
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
Clear() noexcept {
  // Reset the initial state variables of this object:
  m_weight = 0.;
}

// Writes to a standard output stream all of the data for the edge, including
// control signals for nonholonomic robots.
// This function is symmetric with Write().
template <typename GraphType>
void
CompositeEdge<GraphType>::
Write(std::ostream& _os) const {
  // Write the intermediates of the composite edge.
  _os << m_intermediates.size() << " ";
  for(auto& cfg : m_intermediates)
    _os << cfg;
  _os << std::scientific << std::setprecision(16) << m_weight;

  // Clear scientific/precision options.
  _os.unsetf(std::ios_base::floatfield);
}

template <typename GraphType>
std::ostream&
operator<<(std::ostream& _os, const CompositeEdge<GraphType>& _w) {
  _w.Write(_os);
  return _os;
}

/*-------------------------- Individual Local Plans --------------------------*/

template <typename GraphType>
void
CompositeEdge<GraphType>::
SetEdge(Robot* const _robot, const ED _ed) {
  // Make sure that the composite graph has been set
  if(!m_groupMap)
    throw RunTimeException(WHERE) << "Can't set an edge descriptor without a "
                                  << "composite graph.";

  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);
  m_edges[index] = _ed;
}


template <typename GraphType>
void
CompositeEdge<GraphType>::
SetEdge(Robot* const _robot, IndividualEdge&& _edge) {
  const size_t index = m_group->GetGroupIndex(_robot);
  SetEdge(index, std::move(_edge));
}


template <typename GraphType>
void
CompositeEdge<GraphType>::
SetEdge(const size_t robotIndex, IndividualEdge&& _edge) {
  m_localEdges[robotIndex] = std::move(_edge);
  m_edges[robotIndex] = INVALID_ED;
}


template <typename GraphType>
typename CompositeEdge<GraphType>::IndividualEdge*
CompositeEdge<GraphType>::
GetEdge(const size_t _robotIndex) {
  return const_cast<IndividualEdge*>(GetEdge(m_group->GetRobot(_robotIndex)));
}


template <typename GraphType>
const typename CompositeEdge<GraphType>::IndividualEdge*
CompositeEdge<GraphType>::
GetEdge(const size_t _robotIndex) const {
  return GetEdge(m_group->GetRobot(_robotIndex));
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
  const size_t index = m_group->GetGroupIndex(_robot);

  const ED& descriptor = m_edges.at(index);
  if(descriptor != INVALID_ED)
    return &m_groupMap->GetIndividualGraph(index)->GetEdge(descriptor.source(),
                                                   descriptor.target());

  try {
    return &m_localEdges.at(index);
  }
  catch(const std::out_of_range&) {
    throw RunTimeException(WHERE) << "Requested individual edge for robot "
                                  << index << " (" << _robot << "), which is"
                                  << " either stationary for this composite edge"
                                  << "or not in the group.";
  }
}


template <typename GraphType>
std::vector<typename CompositeEdge<GraphType>::ED>&
CompositeEdge<GraphType>::
GetEdgeDescriptors() noexcept {
  return m_edges;
}


template <typename GraphType>
std::vector<typename CompositeEdge<GraphType>::IndividualEdge>&
CompositeEdge<GraphType>::
GetLocalEdges() noexcept {
  return m_localEdges;
}


template <typename GraphType>
void
CompositeEdge<GraphType>::
ClearLocalEdges() noexcept {
  m_localEdges.clear();
}


template <typename GraphType>
size_t
CompositeEdge<GraphType>::
GetNumRobots() const noexcept {
  return m_group->Size();
}

template <typename GraphType>
std::unordered_set<Robot*>
CompositeEdge<GraphType>::
GetActiveRobots() {
  // Note: this only works for non-local edges
  std::unordered_set<Robot*> actives;

  for(size_t i = 0; i < m_edges.size(); ++i) {
    if(m_edges[i] == INVALID_ED)
      continue;

    auto roadmap = m_groupMap->GetIndividualGraph(i);

    typename GraphType::CEI ei;
    typename GraphType::CVI vi;
    roadmap->find_edge(m_edges[i], vi, ei);

    if(ei->source() != ei->target()) {
      actives.insert(m_group->GetRobot(i));
    }
  }
  return actives;
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
  if(!m_groupMap)
    return CompositeEdge(m_group, m_weight + _other.m_weight);
  else
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
