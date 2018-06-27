#include "CDInfo.h"

#include <limits>

CDInfo::
CDInfo(bool _retAllInfo) {
  ResetVars(_retAllInfo);
}

void
CDInfo::
ResetVars(bool _retAllInfo) {
  m_retAllInfo = _retAllInfo;
  m_collidingObstIndex = -1;
  m_nearestObstIndex = -1;
  m_minDist = std::numeric_limits<double>::max();
  m_robotPoint(0, 0, 0);
  m_objectPoint(0, 0, 0);
  m_trianglePairs.clear();
}

bool
CDInfo::
operator<(const CDInfo& _cdInfo) {
  return m_minDist < _cdInfo.m_minDist;
}


CDInfo&
CDInfo::
operator=(const CDInfo& _cdInfo) {
  m_retAllInfo = _cdInfo.m_retAllInfo;
  m_collidingObstIndex = _cdInfo.m_collidingObstIndex;
  m_nearestObstIndex = _cdInfo.m_nearestObstIndex;
  m_minDist = _cdInfo.m_minDist;
  m_robotPoint = _cdInfo.m_robotPoint;
  m_objectPoint = _cdInfo.m_objectPoint;
  m_trianglePairs = _cdInfo.m_trianglePairs;
  // Don't set the self-distance stuff, as that is maintained slightly differently
  return *this;
}

