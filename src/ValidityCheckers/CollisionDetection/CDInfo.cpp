#include "CDInfo.h"

CDInfo::
CDInfo() {
  ResetVars();
}

CDInfo::
~CDInfo() {
}

void CDInfo::ResetVars() {
  m_collidingObstIndex = -1;
  
  m_retAllInfo = false;
  m_nearestObstIndex = -1;
  m_minDist = MaxDist;      // =  1e10 by CollisionDetection.h
  m_robotPoint(0, 0, 0);         // hope Vector3d class defined well
  m_objectPoint(0, 0, 0);
  m_rapidContactID1=-1;
  m_rapidContactID2=-1;
}

