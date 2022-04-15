#ifndef PPL_COMPOSITE_STATE_H_
#define PPL_COMPOSITE_STATE_H_

// #include "ConfigurationSpace/GenericStateGraph.h"
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
/// An aggregate configuration which represents a state for each robot
/// in a robot group.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
class CompositeState {

  public:

    ///@name Local Types
    ///@{

    typedef typename GraphType::CfgType CfgType;
    // typedef typename CompositeState<GraphType> CompositeStateType;
    typedef CompositeEdge<GraphType>  CompositeEdgeType;

    typedef CompositeGraph<CompositeState, CompositeEdgeType> GroupGraphType;

    typedef size_t           VID;      ///< A VID in an individual graph.
    typedef std::vector<VID> VIDSet;   ///< A set of VIDs from indiv. graphs.

    typedef GraphType IndividualGraph;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a group configuration.
    /// @param _groupMap The group roadmap to which this configuration belongs,
    ///                  or null if it is not in a map.
    /// @param _init Default-initialize local configurations?
    /// @todo This object does not work at all without a group map. We should
    ///       throw relevant exceptions if needed.
    explicit CompositeState(GroupGraphType* const _groupGraph = nullptr);

    // virtual ~CompositeState() = 0;

    ///@}
    ///@name Equality
    ///@{

    /// Check if the current and given group configurations are equal.
    /// @param _other The given group configuration.
    /// @return True if equal, false otherwise.
    virtual bool operator==(const CompositeState& _other) const noexcept;

    /// Check if the current and given group configurations are unequal.
    /// @param _other The given group configuration.
    /// @return True if unequal, false otherwise.
    virtual bool operator!=(const CompositeState& _other) const noexcept;

    ///@}
    ///@name Robots
    ///@{
    /// Access the robots within this group configuration.

    /// Get the number of robots.
    virtual size_t GetNumRobots() const noexcept;

    /// Get the full vector of robot pointers.
    virtual const std::vector<Robot*>& GetRobots() const noexcept;

    /// Get the robot pointer for a group member by index.
    /// @param _index The desired index.
    virtual Robot* GetRobot(const size_t _index) const;

    ///@}
    ///@name Roadmap Accessors
    ///@{
    /// These functions provide access to the related group map (if any) and
    /// descriptors for non-local individual configurations.

    /// Get the group roadmap this group cfg is with respect to.
    virtual GroupGraphType* GetGroupGraph() const noexcept;

    /// Get the VID for a particular robot.
    /// @param _index The index (within the group) of the robot.
    /// @return The VID of the robot's individual configuration, or INVALID_VID
    ///         if it is a local configuration.
    virtual VID GetVID(const size_t _index) const noexcept;

    virtual VID GetVID(Robot* const _robot) const;
 
    ///@}
    ///@name Individual Configurations
    ///@{
    /// These functions manage the individual configurations that comprise this
    /// group configuration.

    /// Set the individual cfg for a robot to a roadmap copy of an cfg.
    /// @param _robot The robot which the cfg refers to.
    /// @param _vid The cfg descriptor.
    virtual void SetRobotCfg(Robot* const _robot, const VID _vid);

    /// Set the individual cfg for a robot to a roadmap copy of an cfg.
    /// @param _index The robot's group index which the cfg refers to.
    /// @param _vid The cfg descriptor.
    virtual void SetRobotCfg(const size_t _index, const VID _vid);

    /// Get the individual Cfg for a robot in the group.
    /// @param _robot The robot which the cfg refers to.
    /// @return The individual configuration for the indexed robot.
    virtual CfgType& GetRobotCfg(Robot* const _robot);

    /// Get the individual Cfg for a robot in the group.
    /// @param _index The index of the robot.
    /// @return The individual configuration for the indexed robot.
    virtual CfgType& GetRobotCfg(const size_t _index);

    /// Get the individual Cfg for a robot in the group.
    /// @param _robot The robot which the cfg refers to.
    /// @return The individual configuration for the indexed robot.
    virtual const CfgType& GetRobotCfg(Robot* const _robot) const;

    /// Get the individual Cfg for a robot in the group.
    /// @param _index The index of the robot.
    /// @return The individual configuration for the indexed robot.
    virtual const CfgType& GetRobotCfg(const size_t _index) const;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Verify that an index is valid. Throw an exception if not.
    /// @param _robotIndex The (group) index to verify.
    virtual void VerifyIndex(const size_t _robotIndex) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    GroupGraphType* m_groupGraph{nullptr};  ///< The group graph.

    VIDSet m_vids;   ///< The individual VIDs in this aggregate configuration.

    ///@}

};

template <typename GraphType>
std::ostream& operator<<(std::ostream&, const CompositeState<GraphType>&);

/*------------------------------- Construction -------------------------------*/

template <typename GraphType>
CompositeState<GraphType>::
CompositeState(GroupGraphType* const _groupGraph) : m_groupGraph(_groupGraph) {

  // If no group map was given, this is a placeholder object. We can't do
  // anything with it since every meaningful operation requires a group map.
  if(!m_groupGraph)
    return;

  // Set the VID list to all invalid.
  m_vids.resize(GetNumRobots(), INVALID_VID);
}


/*--------------------------------- Equality ---------------------------------*/

template <typename GraphType>
bool
CompositeState<GraphType>::
operator==(const CompositeState<GraphType>& _other) const noexcept {
  // If _other is from another map, these are not the same.
  if(m_groupGraph != _other.m_groupGraph)
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
  return m_groupGraph ? m_groupGraph->GetGroup()->Size() : 0;
}


template <typename GraphType>
const std::vector<Robot*>&
CompositeState<GraphType>::
GetRobots() const noexcept {
  return m_groupGraph->GetGroup()->GetRobots();
}


template <typename GraphType>
Robot*
CompositeState<GraphType>::
GetRobot(const size_t _index) const {
  VerifyIndex(_index);

  Robot* const robot = m_groupGraph->GetGroup()->GetRobot(_index);

  /// @todo Remove this after we are very sure things are working.
  if(!robot)
    throw RunTimeException(WHERE) << "Error! Robot pointer was null.";

  return robot;
}

/*---------------------------- Roadmap Accessors -----------------------------*/

template <typename GraphType>
typename CompositeState<GraphType>::GroupGraphType*
CompositeState<GraphType>::
GetGroupGraph() const noexcept {
  return m_groupGraph;
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
  const size_t index = m_groupGraph->GetGroup()->GetGroupIndex(_robot);
  return GetVID(index);
}

/*------------------------ Individual Configurations -------------------------*/

template <typename GraphType>
void
CompositeState<GraphType>::
SetRobotCfg(Robot* const _robot, const VID _vid) {
  const size_t index = m_groupGraph->GetGroup()->GetGroupIndex(_robot);
  SetRobotCfg(index, _vid);
}


template <typename GraphType>
void
CompositeState<GraphType>::
SetRobotCfg(const size_t _index, const VID _vid) {
  VerifyIndex(_index);

  m_vids[_index] = _vid;
}


template <typename GraphType>
typename CompositeState<GraphType>::CfgType&
CompositeState<GraphType>::
GetRobotCfg(Robot* const _robot) {
  const size_t index = m_groupGraph->GetGroup()->GetGroupIndex(_robot);
  return GetRobotCfg(index);
}


template <typename GraphType>
typename CompositeState<GraphType>::CfgType&
CompositeState<GraphType>::
GetRobotCfg(const size_t _index) {
  VerifyIndex(_index);

  const VID vid = GetVID(_index);
  if(vid != INVALID_VID)
    return m_groupGraph->GetRoadmap(_index)->GetVertex(vid);
  else
    throw RunTimeException(WHERE) << "Requested Cfg for robot " << _index
                                  << ", but it is invalid.";
}


template <typename GraphType>
const typename CompositeState<GraphType>::CfgType&
CompositeState<GraphType>::
GetRobotCfg(Robot* const _robot) const {
  const size_t index = m_groupGraph->GetGroup()->GetGroupIndex(_robot);
  return GetRobotCfg(index);
}


template <typename GraphType>
const typename CompositeState<GraphType>::CfgType&
CompositeState<GraphType>::
GetRobotCfg(const size_t _index) const {
  VerifyIndex(_index);

  // If we have a valid VID for this robot, fetch its configuration from its
  // individual roadmap.
  const VID vid = GetVID(_index);
  if(vid != INVALID_VID)
    return m_groupGraph->GetRoadmap(_index)->GetVertex(vid);
  else
    throw RunTimeException(WHERE) << "Requested Cfg for robot " << _index
                                  << ", but it is invalid.";
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

  // Loop through all robots in the group and print each one's cfg in order.
  for(size_t i = 0; i < _compositeState.GetNumRobots(); ++i)
    _os << _compositeState.GetRobotCfg(i);

  return _os;
}

/*----------------------------------------------------------------------------*/

#endif
