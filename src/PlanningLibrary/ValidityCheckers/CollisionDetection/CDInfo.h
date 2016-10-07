#ifndef CD_INFO_H_
#define CD_INFO_H_

#include "Vector.h"
#include<vector>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief Information returned by validity checkers, e.g., distance from
///        obstacles.
////////////////////////////////////////////////////////////////////////////////
struct CDInfo {

  //////////////////////////////////////////////////////////////////////////////
  /// @param _retAllInfo Compute distance information if possible
  CDInfo(bool _retAllInfo = false);

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Resets all data to default information
  /// @param _retAllInfo Compute distance information if possible
  void ResetVars(bool _retAllInfo = false);

  //////////////////////////////////////////////////////////////////////////////
  /// @param _cdInfo Other CDInfo
  /// @return Is minimum distance less than other's minimum distance
  bool operator<(const CDInfo& _cdInfo);

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

  // The index for collided robots
  std::vector<std::pair<size_t, size_t>> m_collidingRobtIndex;        
};

#endif
