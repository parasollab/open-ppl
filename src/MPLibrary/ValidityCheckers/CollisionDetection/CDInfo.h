#ifndef CD_INFO_H_
#define CD_INFO_H_

#include <utility>
#include <vector>

#include "Vector.h"


////////////////////////////////////////////////////////////////////////////////
/// Information returned by validity checkers, e.g., distance from obstacles.
///
/// @todo Generalize this object to store collisions with obstacles, boundaries,
///       and other robots in a uniform way. It should contain a vector of
///       'Collision' structures which describe the object type, indexes, and
///       distance for each detected collision.
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
struct CDInfo {

  ///@name Local Types
  ///@{

  /// A pair of triangle indexes on the robot (first) and obstacle (second) in a
  /// discovered collision.
  typedef std::pair<int, int> CollisionPair;

  ///@}
  ///@name Construction
  ///@{

  /// @param _retAllInfo Compute distance information if possible
  CDInfo(const bool _retAllInfo = false);

  /// Reset object to default state.
  /// @param _retAllInfo Compute distance information if possible
  void ResetVars(const bool _retAllInfo = false);

  ///@}
  ///@name Ordering
  ///@{

  /// Order these objects according to the minimum distance from closest
  /// colliding obstacle.
  /// @param _cdInfo Other CDInfo
  /// @return True if minimum distance is less than other's minimum distance.
  bool operator<(const CDInfo& _cdInfo) const noexcept;

  ///@}
  ///@name Internal State
  ///@{

  bool m_retAllInfo;              ///< Consider all collisions or only the first?
  int m_collidingObstIndex;       ///< Index for first discovered obstacle collision.
  int m_nearestObstIndex;         ///< Index for closest obstacle.
  double m_minDist;               ///< Distance between Robot and closest obstacle.

  mathtool::Vector3d m_robotPoint;  ///< Closest point on Robot to closest obstacle.
  mathtool::Vector3d m_objectPoint; ///< Closest point on closest obstacle to Robot.

  std::vector<CollisionPair> m_trianglePairs; ///< All colliding triangle pairs.

  /// Required for assembly planning. In the case of a subassembly, this will be
  /// the closest distance of any robot NOT in the subassembly, compared to any
  /// robot in it.
  std::vector<double> m_selfClearance;

  ///@}

};

#endif
