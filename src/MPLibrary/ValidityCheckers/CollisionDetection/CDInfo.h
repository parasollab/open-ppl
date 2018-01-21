#ifndef CD_INFO_H_
#define CD_INFO_H_

#include <utility>
#include <vector>

#include "Vector.h"


////////////////////////////////////////////////////////////////////////////////
/// Information returned by validity checkers, e.g., distance from obstacles.
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
struct CDInfo {

  /// @param _retAllInfo Compute distance information if possible
  CDInfo(bool _retAllInfo = false);

  /// @brief Resets all data to default information
  /// @param _retAllInfo Compute distance information if possible
  void ResetVars(bool _retAllInfo = false);

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

  typedef std::pair<int, int> CollisionPair;

  std::vector<CollisionPair> m_trianglePairs; ///< All colliding triangle pairs.

  /// m_selfClearance is required for assembly planning. In the case of a
  /// subassembly, this will normally be the closest distance wrt any body of it
  std::vector<double> m_selfClearance; ///< Clearance of robot's bodies to
                                       ///< each other.
};

#endif
