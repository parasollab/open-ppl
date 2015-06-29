#ifndef PQP_COLLISION_DETECTION_H_
#define PQP_COLLISION_DETECTION_H_

#ifdef USE_PQP

#include <PQP.h>

#include "CollisionDetectionMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @brief Proximity Query Package (PQP) collision detection middleware
///
/// Computed collision information with PQP package. PQP has the option to
/// compute clearance and penetration information through distance queries. To
/// enable this pass in @c CDInfo with @c m_retAllInfo set to true.
////////////////////////////////////////////////////////////////////////////////
class PQP : public CollisionDetectionMethod {
  public:
    PQP();
    virtual ~PQP();

    virtual void Build(Body* _body);

    virtual bool IsInCollision(shared_ptr<Body> _body1,
        shared_ptr<Body> _body2, CDInfo& _cdInfo);
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @brief Proximity Query Package (PQP) collision detection middleware
///
/// Computed collision information with PQP package. PQP has the option to
/// compute clearance and penetration information through distance queries. To
/// enable this pass in @c CDInfo with @c m_retAllInfo set to true. PQPSolid
/// additionally can determine if a point lies within an obstacle or not, i.e.,
/// it can be used for @c IsInsideObstacle checks.
////////////////////////////////////////////////////////////////////////////////
class PQPSolid : public PQP {
  public:
    PQPSolid() : PQP() {m_name = "PQP_SOLID";}

    virtual bool IsInCollision(shared_ptr<Body> _body1,
        shared_ptr<Body> _body2, CDInfo& _cdInfo);

    virtual bool IsInsideObstacle(const Vector3d& _pt, shared_ptr<Body> _body);

  private:
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Builds minimal area triangle extending to (_dX, _dY, _dZ) from
    ///        the origin.
    /// @param _dX X coordinate
    /// @param _dY Y coordinate
    /// @param _dZ Z coordinate
    /// @return PQP model of triangle
    PQP_Model* BuildPQPSegment(PQP_REAL _dX, PQP_REAL _dY, PQP_REAL _dZ) const;
};

#endif

#endif
