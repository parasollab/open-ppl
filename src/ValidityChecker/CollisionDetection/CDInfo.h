#ifndef CDINFO_h
#define CDINFO_h

#include "Vector.h"

const double MaxDist =  1e10;

class CDInfo {
 public:
  CDInfo();
  ~CDInfo();
  
  void ResetVars();
  
  int m_collidingObstIndex;   ///< The index for fisrt discovered obstacle which collides with robot.
  bool m_retAllInfo;          ///< Is this instance contains all (following) infomation.
  int m_nearestObstIndex;     ///< The index for closest obstacle
  double m_minDist;            ///< Distance between Robot and closet obstacle
  Vector3D m_robotPoint;       ///< Cloest point on Robot to closet obstacle
  Vector3D m_objectPoint;      ///< Cloest point on closet obstacle to Robot
  ///////////////////////////////////////////////////////////////////////////////////////////
  //for obrrt need contact information
  int m_rapidContactID1;
  int m_rapidContactID2;
};

#endif
