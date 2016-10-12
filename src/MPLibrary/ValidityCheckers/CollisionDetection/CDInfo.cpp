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
  m_rapidContactID1 = -1;
  m_rapidContactID2 = -1;
  m_collidingRobtIndex.clear();
}

bool
CDInfo::
operator<(const CDInfo& _cdInfo) {
  return m_minDist < _cdInfo.m_minDist;
}
