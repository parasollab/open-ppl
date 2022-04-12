#include "CompositeState.h"

#include "MPProblem/RobotGroup/RobotGroup.h"
#include "nonstd.h"


/*------------------------------- Construction -------------------------------*/

template <typename GraphType>
CompositeState<GraphType>::
CompositeState(GraphType* const _groupGraph, 
  CfgType& (*_vertexGetter)(const VID)) : 
  m_groupGraph(_groupGraph), m_vertexGetter(_vertexGetter) {

  // If no group map was given, this is a placeholder object. We can't do
  // anything with it since every meaningful operation requires a group map.
  if(!m_groupGraph or !_vertexGetter)
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
GraphType*
CompositeState<GraphType>::
GetGraph() const noexcept {
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
CfgType&
CompositeState<GraphType>::
GetRobotCfg(Robot* const _robot) {
  const size_t index = m_groupGraph->GetGroup()->GetGroupIndex(_robot);
  return GetRobotCfg(index);
}


template <typename GraphType>
CfgType&
CompositeState<GraphType>::
GetRobotCfg(const size_t _index) {
  VerifyIndex(_index);

  const VID vid = GetVID(_index);
  if(vid != INVALID_VID)
    return m_groupGraph->GetRoadmap(_index)->m_vertexGetter(vid);
}


template <typename GraphType>
const CfgType&
CompositeState<GraphType>::
GetRobotCfg(Robot* const _robot) const {
  const size_t index = m_groupGraph->GetGroup()->GetGroupIndex(_robot);
  return GetRobotCfg(index);
}


template <typename GraphType>
const CfgType&
CompositeState<GraphType>::
GetRobotCfg(const size_t _index) const {
  VerifyIndex(_index);

  // If we have a valid VID for this robot, fetch its configuration from its
  // individual roadmap.
  const VID vid = GetVID(_index);
  if(vid != INVALID_VID)
    return m_groupGraph->GetRoadmap(_index)->m_vertexGetter(vid);
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
