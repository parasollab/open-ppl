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
///
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
    /// @param _g The group roadmap in which this edge exists. Defaults to nullptr.
    /// @param _w The weight of the plan. Defaults to 0.0.
    /// @param _intermediates The intermediates along the edge. Defaults to CompositePath().
    CompositeEdge(GroupGraphType* const & _g = nullptr, const double _w = 0.0, 
      const CompositePath& _intermediates = CompositePath());

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
    ///@name Graph Edge Weight
    ///@{
    /// Get/set the numeric weight for this edge as used when querying a
    /// roadmap.

    virtual EdgeWeight GetWeight() const noexcept;

    virtual void SetWeight(const EdgeWeight _w) noexcept;

    ///@}
    ///@name Misc. Interface Functions
    ///@{

    // There is no current use case where these should ever get reset to false.
    virtual void SetSkipEdge() noexcept;
    virtual bool SkipEdge() const noexcept;
    
    /// Reset the states of this object.
    virtual void Clear() noexcept;

    /// Get the composite state intermediates.
    CompositePath& GetIntermediates() noexcept;
    /// Get the composite state intermediates.
    const CompositePath& GetIntermediates() const noexcept;

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

    /// overload for Robot pointer
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

    ///Overloads for using index instead of robot pointer.
    virtual IndividualEdge* GetEdge(const size_t _robotIndex);
    virtual const IndividualEdge* GetEdge(const size_t _robotIndex) const;

    /// Get a vector of local edges in the plan.
    std::vector<IndividualEdge>& GetLocalEdges() noexcept;

    /// Clear all local edges in the plan.
    void ClearLocalEdges() noexcept;

    /// Get a vector of local edges' descriptors.
    virtual std::vector<ED>& GetEdgeDescriptors() noexcept;

    /// Get the number of robots given in this group local plan.
    virtual size_t GetNumRobots() const noexcept;

    virtual std::unordered_set<Robot*> GetActiveRobots();

    virtual void SetTimeSteps(size_t _timesteps);

    virtual size_t GetTimeSteps() const;

    ///@}
    ///@name Stapl graph interface
    ///@{

    /// This only adds weights, it doesn't take intermediates into account.
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

    GroupGraphType* m_groupMap{nullptr};  ///< The composite graph that this edge is in.

    /// The edge weight.
    double m_weight{std::numeric_limits<double>::infinity()};

    CompositePath m_intermediates; ///< Group cfg intermediates.

    std::vector<ED> m_edges;           ///< Descriptors of the individual edges.

    /// Note that any edges added to m_localEdges must be valid and complete.
    std::vector<IndividualEdge> m_localEdges; ///< Edges which are not in a map.

    bool m_skipEdge{false}; ///< Flag to skip full recreation in GroupPath::FullCfgs.

    size_t m_timesteps;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename GraphType>
CompositeEdge<GraphType>::
CompositeEdge(GroupGraphType* const & _g, const double _w, 
    const CompositePath& _intermediates)
    : m_groupMap(_g), m_weight(_w), m_intermediates(_intermediates) {
  if(m_groupMap)
    m_edges.resize(m_groupMap->GetGroup()->Size(), INVALID_ED);
}

/*------------------------- Misc Interface Functions -------------------------*/

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
  for(size_t i = 0; i < m_groupMap->GetGroup()->Size(); ++i) {
  // for(const size_t i : m_groupMap->GetGroup()->Size()) {
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

// Writes to a standard output stream all of the data for the edge, including
// control signals for nonholonomic robots.
// This function is symmetric with Write().
template <typename GraphType>
void
CompositeEdge<GraphType>::
Write(std::ostream& _os) const {
  /// @TODO Now that we read/write the control signals for nonholonomic, we
  ///  should remove the writing of intermediates for conciseness.

  //Write the data that's needed whether it's a holonomic robot or not:
  _os << m_intermediates.size() << " ";
  for(auto&  cfg : m_intermediates)
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
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);

  m_edges[index] = _ed;
}


template <typename GraphType>
void
CompositeEdge<GraphType>::
SetEdge(Robot* const _robot, IndividualEdge&& _edge) {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);
  SetEdge(index, std::move(_edge));
}


template <typename GraphType>
void
CompositeEdge<GraphType>::
SetEdge(const size_t robotIndex, IndividualEdge&& _edge) {
  // Allocate space for local edges if not already done.
  m_localEdges.resize(m_groupMap->GetGroup()->Size());

  m_localEdges[robotIndex] = std::move(_edge);
  m_edges[robotIndex] = INVALID_ED;
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
  return GetEdge(m_groupMap->GetGroup()->GetRobot(_robotIndex));
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
  return m_groupMap->GetGroup()->Size();
}

template <typename GraphType>
std::unordered_set<Robot*>
CompositeEdge<GraphType>::
GetActiveRobots() {
  std::unordered_set<Robot*> actives;

  for(size_t i = 0; i < m_edges.size(); ++i) {
    auto roadmap = m_groupMap->GetRoadmap(i);

    typename GraphType::CEI ei;
    typename GraphType::CVI vi;
    roadmap->find_edge(m_edges[i], vi, ei);

    if(ei->source() != ei->target()) {
      actives.insert(m_groupMap->GetGroup()->GetRobot(i));
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
