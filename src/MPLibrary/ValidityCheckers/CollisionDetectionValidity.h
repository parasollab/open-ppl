#ifndef PMPL_COLLISION_DETECTION_VALIDITY_H_
#define PMPL_COLLISION_DETECTION_VALIDITY_H_

#include "MPLibrary/ValidityCheckers/CollisionDetectionValidityMethod.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/Environment/Environment.h"

#include "Utilities/MetricUtils.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/RapidCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/SpheresCollisionDetection.h"

#include "nonstd/io.h"

#include <algorithm>


////////////////////////////////////////////////////////////////////////////////
/// Classifies validity based on collisions with other objects in the workspace.
///
/// There are three types of collision that can occur:
/// 1. Self collision. Two or more pieces of the robot's geometry are
///    overlapping in workspace.
/// 2. Obstacle collision. The robot's geometry overlaps with some workspace
///    obstacle.
/// 3. Inter-robot collision. The robot's geometry overlaps with some other
///    robot in its current configuration.
/// For group configurations, collisions between robots within the group are
/// considered as self-collisions, while collisions with robots not in the group
/// are classified as inter-robot collisions. This distinction is made because
/// considering self-collisions usually means that we want (group)
/// configurations which are valid without considering the obstacles and other
/// robots.
///
/// Any robot can be omitted from the collision checks by setting it as virtual.
/// When collision checking a virtual robot, other robots are also ignored.
///
/// @note This class interfaces with external CD libraries to determine
///       collision information, sometimes including clearance and penetration
///       information.
///
/// @todo Remove the 'GetCDMethod' function after re-implementing
///       ObstacleClearanceValidity as a subtype of CollisionDetectionValidity.
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
class CollisionDetectionValidity 
  : public CollisionDetectionValidityMethod {

  public:

    ///@name Local Types
    ///@{

    typedef typename CollisionDetectionValidityMethod::GroupCfg GroupCfg;
    typedef typename GroupCfg::Formation    Formation;

    ///@}
    ///@name Construction
    ///@{

    CollisionDetectionValidity();

    CollisionDetectionValidity(XMLNode& _node);

    virtual ~CollisionDetectionValidity();

    ///@}
    ///@name CollisionDetection Interface
    ///@{

    /// @return Collision Detection object
    virtual CollisionDetectionMethod* GetCDMethod() const noexcept override;

    /// Determine whether a workspace point lies inside of an obstacle.
    /// @param _p The workspace point.
    /// @return True if _p is inside an obstacle.
    virtual bool IsInsideObstacle(const Point3d& _p) override;

    /// Check if two workspace points are mutually visible.
    /// @param _a The first point.
    /// @param _b The second point.
    /// @return True if _a is visible from _b and vice versa.
    virtual bool WorkspaceVisibility(const Point3d& _a, const Point3d& _b) override;

    /// Check for collision between two multibodies.
    /// @param _cdInfo CDInfo
    /// @param _a The first multibody.
    /// @param _b The second multibody.
    /// @param _caller Function calling validity checker.
    /// @return True if _a and _b collide in their present configurations.
    virtual bool IsMultiBodyCollision(CDInfo& _cdInfo, const MultiBody* const _a,
        const MultiBody* const _b, const std::string& _caller) override;

    ///@}

  protected:

    ///@name ValidityCheckerMethod Overrides
    ///@{

    virtual bool IsValidImpl(Cfg& _cfg, CDInfo& _cdInfo,
        const std::string& _caller) override;

    virtual bool IsValidImpl(GroupCfg& _cfg, CDInfo& _cdInfo,
        const std::string& _caller) override;

    ///@}
    ///@name Helpers
    ///@{

    /// Orchestrate collision computation between robot and environment
    /// multibodies
    /// @param _cdInfo Output for collision detection info.
    /// @param _cfg Configuration of interest.
    /// @param _caller Name of the calling function.
    /// @return True if the robot is in collision at _cfg.
    virtual bool IsInCollision(CDInfo& _cdInfo, const Cfg& _cfg,
        const std::string& _caller);

    /// Orchestrate collision computation between robots in a group cfg
    /// and environment multibodies
    /// @param _cdInfo Output for collision detection info.
    /// @param _cfg Group configuration of interest.
    /// @param _caller Name of the calling function.
    /// @return True if the robot group is in collision at _cfg.
    virtual bool IsInCollision(CDInfo& _cdInfo, const GroupCfg& _cfg,
        const std::string& _caller);

    /// Check if any of the robot's bodies are in collision with each other.
    /// @param _cdInfo Output for collision detection info. It will only be
    ///                updated if the detected collision is closer than the
    ///                previous.
    /// @param _multibody The robot's MultiBody.
    /// @param _caller Name of the calling function.
    /// @return True if the robot is in self-collision.
    virtual bool IsInSelfCollision(CDInfo& _cdInfo,
        const MultiBody* const _multibody, const std::string& _caller);

    /// Check if any of the robot's bodies are in collision with or outside the
    /// environment boundary.
    /// @param _cdInfo Output for collision detection info. It will only be
    ///                updated if the detected collision is closer than the
    ///                previous.
    /// @param _cfg The robot configuration.
    /// @return True if the robot is in self-collision.
    virtual bool IsInBoundaryCollision(CDInfo& _cdInfo, const Cfg& _cfg);

    /// Check if any of the robot's bodies are in collision with an obstacle.
    /// @param _cdInfo Output for collision detection info. It will only be
    ///                updated if the detected collision is closer than the
    ///                previous.
    /// @param _multibody The robot's MultiBody.
    /// @param _caller Name of the calling function.
    /// @return True if the robot is in collision with an obstacle.
    virtual bool IsInObstacleCollision(CDInfo& _cdInfo,
        const MultiBody* const _multibody, const std::string& _caller);

    /// Check for a collision between a query robot and a specific set of other
    /// robots.
    /// @param _robot The query robot.
    /// @param _robots The other robots.
    /// @param _caller Name of the calling function.
    virtual bool IsInInterRobotCollision(CDInfo& _cdInfo, Robot* const _robot,
        const std::vector<Robot*>& _robots, const std::string& _caller);

    ///@}
    ///@name Internal State
    ///@{

    ///< Underlying collision detection object.
    std::unique_ptr<CollisionDetectionMethod> m_cdMethod;

    bool m_ignoreSelfCollision{false};    ///< Check self collisions
    bool m_interRobotCollision{false};    ///< Check inter-robot collisions
    bool m_ignoreAdjacentLinks{false};    ///< Ignore adj links in self collisions
    bool m_ignoreSiblingCollisions{false}; ///< Ignore sibling links in self collisions

    ///@}

};

#endif
