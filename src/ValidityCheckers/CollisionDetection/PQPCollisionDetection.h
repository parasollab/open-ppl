#ifndef PQPCOLLISIONDETECTION_H_
#define PQPCOLLISIONDETECTION_H_

#ifdef USE_PQP
#include "CollisionDetectionMethod.h"
#include <PQP.h>

class PQP : public CollisionDetectionMethod {
  public:
    PQP();
    virtual ~PQP();

    virtual bool IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle,
        StatClass& _stats, CDInfo& _cdInfo, std::string* _callName=NULL, int _ignoreIAdjacentMultibodies=1);
};

class PQPSolid : public PQP {
  public:
    PQPSolid() : PQP(){m_name = "PQP_SOLID";}
    virtual ~PQPSolid() {}

    virtual bool IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle,
        StatClass& _stats, CDInfo& _cdInfo,std::string* _callName=NULL, int ignoreIAdjacentMultibodies=1);

    virtual bool IsInsideObstacle(const Cfg& _cfg, Environment* _env);
    virtual bool IsInsideObstacle(Vector3d _robotPt, shared_ptr<MultiBody> _obstacle);

    PQP_Model* BuildPQPSegment(PQP_REAL _dX, PQP_REAL _dY, PQP_REAL _dZ) const;
};
#endif

#endif
