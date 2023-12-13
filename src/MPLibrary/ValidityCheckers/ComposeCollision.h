#ifndef PMPL_COMPOSE_COLLISION_H_
#define PMPL_COMPOSE_COLLISION_H_

#include "CollisionDetectionValidityMethod.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/Environment/Environment.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"


////////////////////////////////////////////////////////////////////////////////
/// Composed collision detector which applies two or more detection conditions.
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
class ComposeCollision : public CollisionDetectionValidityMethod {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType GroupCfg;

    enum LogicalOperator {AND, OR}; ///< The supported logical operators.

    ///@}
    ///@name Construction
    ///@{

    ComposeCollision();

    ComposeCollision(XMLNode& _node);

    virtual ~ComposeCollision() = default;

    ///@}
    ///@name CollisionDetection Interface
    ///@{

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
    ///@name Internal State
    ///@{

    LogicalOperator m_operator; ///< The logical operator joining CD's.
    std::vector<std::string> m_cdLabels; ///< The CD labels to combine.

    ///@}
};

#endif
