#ifndef CD_INFO_H_
#define CD_INFO_H_

#include "Vector.h"

class CDInfo {
 public:
  CDInfo();

  void ResetVars();

  int m_collidingObstIndex;         ///< The index for fisrt discovered obstacle which collides with robot.
  bool m_retAllInfo;                ///< If this instance contains all (following) information.
  int m_nearestObstIndex;           ///< The index for closest obstacle
  double m_minDist;                 ///< Distance between Robot and closest obstacle
  mathtool::Vector3d m_robotPoint;  ///< Closest point on Robot to closest obstacle
  mathtool::Vector3d m_objectPoint; ///< Closest point on closest obstacle to Robot
  ///////////////////////////////////////////////////////////////////////////////////////////
  //for obrrt need contact information
  int m_rapidContactID1;
  int m_rapidContactID2;
};

#endif
