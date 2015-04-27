#include "CDInfo.h"
#include <limits>

CDInfo::
CDInfo() {
  ResetVars();
}

void
CDInfo::
ResetVars() {
  m_collidingObstIndex = -1;

  m_retAllInfo = false;
  m_nearestObstIndex = -1;
  m_minDist = std::numeric_limits<double>::max();
  m_robotPoint(0, 0, 0);
  m_objectPoint(0, 0, 0);
  m_rapidContactID1=-1;
  m_rapidContactID2=-1;
}

