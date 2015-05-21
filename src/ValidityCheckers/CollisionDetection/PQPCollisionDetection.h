#ifndef PQPCOLLISIONDETECTION_H_
#define PQPCOLLISIONDETECTION_H_

#ifdef USE_PQP
#include <PQP.h>

#include <Vector.h>
using namespace mathtool;

#include "CollisionDetectionMethod.h"

class Environment;

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

    virtual bool IsInCollision(shared_ptr<ActiveMultiBody> _robot,
        shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
        size_t _ignoreIAdjacentMultibodies);
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

    virtual bool IsInCollision(shared_ptr<ActiveMultiBody> _robot,
        shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
        size_t ignoreIAdjacentMultibodies);

    virtual bool IsInsideObstacle(const Cfg& _cfg, Environment* _env);
    virtual bool IsInsideObstacle(Vector3d _robotPt, shared_ptr<MultiBody> _obstacle);

    PQP_Model* BuildPQPSegment(PQP_REAL _dX, PQP_REAL _dY, PQP_REAL _dZ) const;
};
#endif

#endif
