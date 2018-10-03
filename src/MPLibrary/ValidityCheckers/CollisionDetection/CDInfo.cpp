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
  m_selfClearance.clear();
}


bool
CDInfo::
operator<(const CDInfo& _cdInfo) const noexcept {
  return m_minDist < _cdInfo.m_minDist;
}
