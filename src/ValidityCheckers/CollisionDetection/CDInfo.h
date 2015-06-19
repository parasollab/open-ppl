#ifndef CD_INFO_H_
#define CD_INFO_H_

#include "Vector.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief Information returned by validity checkers, e.g., distance from
///        obstacles.
////////////////////////////////////////////////////////////////////////////////
struct CDInfo {
  CDInfo();

  void ResetVars();

  bool m_retAllInfo;                ///< If this instance contains all
                                    ///< (following) information.
  int m_collidingObstIndex;         ///< The index for fisrt discovered obstacle
                                    ///< which collides with robot.
  int m_nearestObstIndex;           ///< The index for closest obstacle
  double m_minDist;                 ///< Distance between Robot and closest
                                    ///< obstacle
  mathtool::Vector3d m_robotPoint;  ///< Closest point on Robot to closest
                                    ///< obstacle
  mathtool::Vector3d m_objectPoint; ///< Closest point on closest obstacle to
                                    ///< Robot
  int m_rapidContactID1;            ///< Triangle on robot in collision
  int m_rapidContactID2;            ///< Triangle on obstacle in collision
};

#endif
