#ifndef PQP_COLLISION_DETECTION_H_
#define PQP_COLLISION_DETECTION_H_

#ifndef NO_PQP

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
    virtual ~PQP() = default;

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

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Shoot a pseudo-ray outward from a reference point to determine if
    ///        it lies within a given body.
    /// @param[in] _pt The reference point of interest.
    /// @param[in] _body The body to check against.
    /// @return True if _pt is inside _body.
    virtual bool IsInsideObstacle(const Vector3d& _pt, shared_ptr<Body> _body);

  private:

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Build a zero-area triangle with base at the origin and extending
    ///        to (+inf, 0, 0) for the ray-shooting test in IsInsideObstacle.
    /// @return A PQP model of a triangular pseudo-ray.
    PQP_Model* BuildPseudoRay() const;

};

#endif

#endif
