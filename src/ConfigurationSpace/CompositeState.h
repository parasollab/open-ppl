#ifndef PPL_COMPOSITE_STATE_H_
#define PPL_COMPOSITE_STATE_H_

#include "ConfigurationSpace/CompositeGraph.h"
#include "ConfigurationSpace/CompositeEdge.h"

#include "MPProblem/RobotGroup/RobotGroup.h"
#include "nonstd.h"

#include <cstddef>
#include <iostream>
#include <vector>

#include "Vector.h"

class Robot;
class RobotGroup;
template <typename GraphType> class CompositeEdge;


////////////////////////////////////////////////////////////////////////////////
/// An aggregate state which represents a state for each robot in a robot
/// group. 'GraphType' represents the individual graph type for a single robot.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
class CompositeState {

  public:

    ///@name Local Types
    ///@{

    typedef size_t           VID;      ///< A VID in an individual graph.
    typedef std::vector<VID> VIDSet;   ///< A set of VIDs from indiv. graphs.
    typedef typename GraphType::CfgType CfgType; ///< The indiv. graph vertex type.

    typedef CompositeEdge<GraphType>                          CompositeEdgeType;
    typedef CompositeGraph<CompositeState, CompositeEdgeType> GroupGraphType;

    typedef GraphType IndividualGraph;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a composite state.
    /// @param _groupGraph The composite graph to which this state belongs.
    explicit CompositeState(GroupGraphType* const _groupGraph = nullptr);

    /// Construct a composite state.
    /// @param _group The robot group to which this state belongs if it is not
    ///               in a graph.
    explicit CompositeState(RobotGroup* const _group);

    ///@}
    ///@name Equality
    ///@{

    /// Check if the current and given composite states are equal.
    /// @param _other The given composite state.
    /// @return True if equal, false otherwise.
    virtual bool operator==(const CompositeState& _other) const noexcept;

    /// Check if the current and given composite states are unequal.
    /// @param _other The given composite state.
    /// @return True if unequal, false otherwise.
    virtual bool operator!=(const CompositeState& _other) const noexcept;

    ///@}
    ///@name Robots
    ///@{
    /// Access the robots within this composite state.

    /// Get the number of robots in this composite state.
    virtual size_t GetNumRobots() const noexcept;

    /// Get the full vector of robot pointers.
    virtual const std::vector<Robot*>& GetRobots() const noexcept;

    /// Get the robot pointer for a group member by index.
    /// @param _index The desired index.
    virtual Robot* GetRobot(const size_t _index) const;

    ///@}
    ///@name Graph Accessors
    ///@{
    /// These functions provide access to the related group graph (if any) and
    /// descriptors for individual states.

    /// Get the group graph this composite state is with respect to.
    virtual GroupGraphType* GetGroupGraph() const noexcept;

    /// Set the composite graph that this composite state exists within.
    virtual void SetGroupGraph(GroupGraphType* _newGraph);

    /// Get the VID for a particular robot.
    /// @param _index The index (within the group) of the robot.
    /// @return The VID of the robot's individual state, or INVALID_VID
    ///         if it is not a valid VID in the invididual graph.
    virtual VID GetVID(const size_t _index) const noexcept;

    /// Get the VID for a particular robot.
    /// @param _robot The a robot within the group.
    /// @return The VID of the robot's individual state, or INVALID_VID
    ///         if it is not a valid VID in the invididual graph.
    virtual VID GetVID(Robot* const _robot) const;
 
    ///@}
    ///@name Individual States
    ///@{
    /// These functions manage the individual states that comprise this
    /// composite state.

    /// Set the individual state (cfg) for a robot to a graph copy of a state.
    /// @param _robot The robot which the state refers to.
    /// @param _vid The state descriptor.
    virtual void SetRobotCfg(Robot* const _robot, const VID _vid);

    /// Set the individual state (cfg) for a robot to a graph copy of a state.
    /// @param _index The robot's group index which the state refers to.
    /// @param _vid The state descriptor.
    virtual void SetRobotCfg(const size_t _index, const VID _vid);

    /// Set the individual state (cfg) for a robot to a graph copy of a state.
    /// @param _robot The robot which the state refers to.
    /// @param _vid The state descriptor.
    virtual void SetRobotCfg(Robot* const _robot, CfgType&& _cfg);

    /// Set the individual state (cfg) for a robot to a graph copy of a state.
    /// @param _index The robot's group index which the state refers to.
    /// @param _vid The state descriptor.
    virtual void SetRobotCfg(const size_t _index, CfgType&& _cfg);

    /// Get the individual state (cfg) for a robot in the group.
    /// @param _robot The robot which the state refers to.
    /// @return The individual state for the indexed robot.
    virtual CfgType& GetRobotCfg(Robot* const _robot);

    /// Get the individual state (cfg) for a robot in the group.
    /// @param _index The index of the robot.
    /// @return The individual state for the indexed robot.
    virtual CfgType& GetRobotCfg(const size_t _index);

    /// Get the individual state (cfg) for a robot in the group.
    /// @param _robot The robot which the state refers to.
    /// @return The individual state for the indexed robot.
    virtual const CfgType& GetRobotCfg(Robot* const _robot) const;

    /// Get the individual state (cfg) for a robot in the group.
    /// @param _index The index of the robot.
    /// @return The individual state for the indexed robot.
    virtual const CfgType& GetRobotCfg(const size_t _index) const;

    /// Clear the Local Cfg information.
    void ClearLocalCfgs();

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Verify that an index is valid. Throw an exception if not.
    /// @param _robotIndex The (group) index to verify.
    virtual void VerifyIndex(const size_t _robotIndex) const noexcept;

    /// Return whether the cfg for the robot is local to the group cfg, or if
    /// it's in an individual roadmap already.
    virtual bool IsLocalCfg(const size_t _robotIndex) const noexcept;

    /// Initialize the set of local configurations if not already done.
    virtual void InitializeLocalCfgs() noexcept;

    ///@}
    ///@name Internal State
    ///@{

    GroupGraphType* m_groupMap{nullptr};  ///< The group graph.

    RobotGroup* m_group{nullptr}; ///< The robot group for this state.

    VIDSet m_vids;   ///< The individual VIDs in this aggregate state.

    std::vector<CfgType> m_localCfgs; ///< Individual states not in a map.

    ///@}

};

template <typename GraphType>
std::ostream& operator<<(std::ostream&, const CompositeState<GraphType>&);

/*------------------------------- Construction -------------------------------*/

template <typename GraphType>
CompositeState<GraphType>::
CompositeState(GroupGraphType* const _groupGraph) : m_groupMap(_groupGraph) {

  // If no group graph was given, this is a placeholder object.
  if(!m_groupMap)
    return;

  // Set the group to the one given in the composite graph.
  m_group = m_groupMap->GetGroup();

  // Set the VID list to all invalid.
  m_vids.resize(GetNumRobots(), INVALID_VID);

  // Initialize local configurations.
  InitializeLocalCfgs();
}


template <typename GraphType>
CompositeState<GraphType>::
CompositeState(RobotGroup* const _group) : m_group(_group) {
  // Set the VID list to all invalid.
  m_vids.resize(GetNumRobots(), INVALID_VID);

  // Initialize local configurations.
  InitializeLocalCfgs();
}


/*--------------------------------- Equality ---------------------------------*/

template <typename GraphType>
bool
CompositeState<GraphType>::
operator==(const CompositeState<GraphType>& _other) const noexcept {
  // If _other is for another group, these are not the same.
  if(m_group != _other.m_group)
    return false;

  // If _other is from another graph, these are not the same.
  if(m_groupMap != _other.m_groupMap)
    return false;

  // Else, compare VIDs if both are valid, or by-value other wise.
  for(size_t i = 0; i < m_vids.size(); ++i) {
    const VID thisVID  = m_vids[i],
              otherVID = _other.m_vids[i];

    if(thisVID != INVALID_VID and otherVID != INVALID_VID) {
      if(thisVID != otherVID)
        return false;
    }
    else if(GetRobotCfg(i) != _other.GetRobotCfg(i))
      return false;
  }

  return true;
}


template <typename GraphType>
bool
CompositeState<GraphType>::
operator!=(const CompositeState<GraphType>& _other) const noexcept {
  return !(*this == _other);
}

/*---------------------------------- Robots ----------------------------------*/

template <typename GraphType>
size_t
CompositeState<GraphType>::
GetNumRobots() const noexcept {
  return m_group ? m_group->Size() : 0;
}


template <typename GraphType>
const std::vector<Robot*>&
CompositeState<GraphType>::
GetRobots() const noexcept {
  return m_group->GetRobots();
}


template <typename GraphType>
Robot*
CompositeState<GraphType>::
GetRobot(const size_t _index) const {
  VerifyIndex(_index);

  Robot* const robot = m_group->GetRobot(_index);

  /// @todo Remove this after we are very sure things are working.
  if(!robot)
    throw RunTimeException(WHERE) << "Error! Robot pointer was null.";

  return robot;
}

/*------------------------------ Graph Accessors -----------------------------*/

template <typename GraphType>
typename CompositeState<GraphType>::GroupGraphType*
CompositeState<GraphType>::
GetGroupGraph() const noexcept {
  return m_groupMap;
}

template <typename GraphType>
void
CompositeState<GraphType>::
SetGroupGraph(GroupGraphType* const _newGraph) {
  // Check that groups are compatible.
  if(m_group != _newGraph->GetGroup())
    throw RunTimeException(WHERE) << "Trying to change graphs on incompatible "
                                  << "groups!";

  // Set the group graph and set all vids to invalid.
  m_groupMap = _newGraph;
  m_vids.resize(GetNumRobots(), INVALID_VID);
}

template <typename GraphType>
typename CompositeState<GraphType>::VID
CompositeState<GraphType>::
GetVID(const size_t _index) const noexcept {
  VerifyIndex(_index);
  return m_vids[_index];
}

template <typename GraphType>
typename CompositeState<GraphType>::VID
CompositeState<GraphType>::
GetVID(Robot* const _robot) const {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);
  return GetVID(index);
}

/*------------------------ Individual States -------------------------*/

template <typename GraphType>
void
CompositeState<GraphType>::
SetRobotCfg(Robot* const _robot, const VID _vid) {
  if(!m_groupMap)
    throw RunTimeException(WHERE) << "Can't set a VID without a composite graph.";

  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);
  SetRobotCfg(index, _vid);
}


template <typename GraphType>
void
CompositeState<GraphType>::
SetRobotCfg(const size_t _index, const VID _vid) {
  if(!m_groupMap)
    throw RunTimeException(WHERE) << "Can't set a VID without a composite graph.";

  VerifyIndex(_index);
  m_vids[_index] = _vid;
}


template <typename GraphType>
void
CompositeState<GraphType>::
SetRobotCfg(Robot* const _robot, CfgType&& _cfg) {
  const size_t index = m_group->GetGroupIndex(_robot);
  SetRobotCfg(index, std::move(_cfg));
}


template <typename GraphType>
void
CompositeState<GraphType>::
SetRobotCfg(const size_t _index, CfgType&& _cfg) {
  VerifyIndex(_index);

  m_localCfgs[_index] = std::move(_cfg);
  m_vids[_index] = INVALID_VID;
}


template <typename GraphType>
void
CompositeState<GraphType>::
ClearLocalCfgs() {
  m_localCfgs.clear();
}


template <typename GraphType>
typename CompositeState<GraphType>::CfgType&
CompositeState<GraphType>::
GetRobotCfg(Robot* const _robot) {
  const size_t index = m_group->GetGroupIndex(_robot);
  return GetRobotCfg(index);
}


template <typename GraphType>
typename CompositeState<GraphType>::CfgType&
CompositeState<GraphType>::
GetRobotCfg(const size_t _index) {
  VerifyIndex(_index);

  const VID vid = GetVID(_index);
  if(vid != INVALID_VID)
    return m_groupMap->GetIndividualGraph(_index)->GetVertex(vid);
  else {
    InitializeLocalCfgs();
    return m_localCfgs[_index];
  }
}


template <typename GraphType>
const typename CompositeState<GraphType>::CfgType&
CompositeState<GraphType>::
GetRobotCfg(Robot* const _robot) const {
  const size_t index = m_group->GetGroupIndex(_robot);
  return GetRobotCfg(index);
}


template <typename GraphType>
const typename CompositeState<GraphType>::CfgType&
CompositeState<GraphType>::
GetRobotCfg(const size_t _index) const {
  VerifyIndex(_index);

  // If we have a valid VID for this robot, fetch its state from its
  // individual graph.
  const VID vid = GetVID(_index);
  if(vid != INVALID_VID)
    return m_groupMap->GetIndividualGraph(_index)->GetVertex(vid);
  
  try {
    return m_localCfgs.at(_index);
  }
  catch(const std::out_of_range&) {
    throw RunTimeException(WHERE) << "Requested configuration for robot "
                                  << _index
                                  << ", but no roadmap or local cfg exists.";
  }
}

/*----------------------------------------------------------------------------*/

template <typename GraphType>
inline
void
CompositeState<GraphType>::
VerifyIndex(const size_t _robotIndex) const noexcept {
  if(_robotIndex >= GetNumRobots())
    throw RunTimeException(WHERE) << "Requested data for robot " << _robotIndex
                                  << ", but the group has only " << GetNumRobots()
                                  << " robots.";
}

/*----------------------------------------------------------------------------*/

template <typename GraphType>
std::ostream&
operator<<(std::ostream& _os, const CompositeState<GraphType>& _compositeState) {
  // Might not need to be hidden behind GROUP_MAP, but doing for consistency
#ifdef GROUP_MAP
  _os << "0 ";
#endif

  // Loop through all robots in the group and print each one's state in order.
  for(size_t i = 0; i < _compositeState.GetNumRobots(); ++i)
    _os << _compositeState.GetRobotCfg(i);

  return _os;
}

/*----------------------------------------------------------------------------*/

template <typename GraphType>
bool
CompositeState<GraphType>::
IsLocalCfg(const size_t _robotIndex) const noexcept {
  // Only true if there is local data (meaning INVALID_VID is present)
  return m_vids[_robotIndex] == INVALID_VID;
}


template <typename GraphType>
void
CompositeState<GraphType>::
InitializeLocalCfgs() noexcept {
  // We will assume the local cfgs are initialized if the container size is
  // correct.
  const size_t numRobots = GetNumRobots();
  if(m_localCfgs.size() == numRobots)
    return;

  m_localCfgs.clear();
  m_localCfgs.resize(numRobots);

  for(size_t i = 0; i < numRobots; ++i)
    m_localCfgs[i] = CfgType(this->GetRobot(i));
}

/*----------------------------------------------------------------------------*/

#endif
