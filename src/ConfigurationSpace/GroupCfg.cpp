#include "GroupCfg.h"

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "MPProblem/RobotGroup/RobotGroup.h"


/*------------------------------- Construction -------------------------------*/

GroupCfg::
GroupCfg(GraphType* const _groupMap) : m_groupMap(_groupMap) {} //,
  //m_vids(_groupMap->GetGroup()->Size()) { }

/*--------------------------------- Equality ---------------------------------*/

bool
GroupCfg::
operator==(const GroupCfg& _other) const noexcept {
  // If _other is from another map, these are not the same.
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


bool
GroupCfg::operator!=(const GroupCfg& _other) const noexcept {
  return !(*this == _other);
}

/*-------------------------------- Accessors ---------------------------------*/

void
GroupCfg::
SetCfg(Robot* const _robot, Cfg&& _cfg) {
  // Allocate space for local cfgs if not already done.
  m_localCfgs.resize(m_groupMap->GetGroup()->Size());

  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);

  m_localCfgs[index] = std::move(_cfg);
  m_vids[index] = INVALID_VID;
}


void
GroupCfg::
SetCfg(Robot* const _robot, const VID _vid) {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);

  m_vids[index] = _vid;
}


Cfg&
GroupCfg::
GetRobotCfg(const size_t _index) {
  return const_cast<Cfg&>(GetRobotCfg(_index));
}


const Cfg&
GroupCfg::
GetRobotCfg(const size_t _index) const {
  const VID vid = GetVID(_index);

  // If we have a valid VID for this robot, fetch its configuration from its
  // individual roadmap.
  if(vid != INVALID_VID) {
    auto roadmap = m_groupMap->GetRoadmap(_index);
    return roadmap->GetVertex(vid);
  }

  try {
    return m_localCfgs.at(_index);
  }
  catch(const std::runtime_error&) {
    std::ostringstream oss;
    oss << "Requested configuration for robot " << _index
        << ", but no roadmap or local cfg exists.";
    throw RunTimeException(WHERE, oss.str());
  }
}


GroupCfg::VID
GroupCfg::
GetVID(const size_t _index) const noexcept {
  return m_vids[_index];
}

/*----------------------------------------------------------------------------*/
