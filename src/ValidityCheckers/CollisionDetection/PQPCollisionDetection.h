#ifndef PQP_COLLISION_DETECTION_H_
#define PQP_COLLISION_DETECTION_H_

#ifdef USE_PQP
#include <PQP.h>

#include "CollisionDetectionMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @brief TODO
///
/// TODO
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
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class PQPSolid : public PQP {
  public:
    PQPSolid() : PQP() {m_name = "PQP_SOLID";}

    virtual bool IsInCollision(shared_ptr<Body> _body1,
        shared_ptr<Body> _body2, CDInfo& _cdInfo);

    virtual bool IsInsideObstacle(const Vector3d& _pt, shared_ptr<Body> _body);

  private:
    PQP_Model* BuildPQPSegment(PQP_REAL _dX, PQP_REAL _dY, PQP_REAL _dZ) const;
};

#endif

#endif
