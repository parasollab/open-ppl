#ifndef PQPCOLLISIONDETECTION_H
#define PQPCOLLISIONDETECTION_H

#ifdef USE_PQP
#include "CollisionDetectionMethod.h"
#include <PQP.h>

class PQP : public CollisionDetectionMethod {
 public:
  PQP();
  virtual ~PQP();

  virtual bool IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, 
			     StatClass& _stats, CDInfo& _cdInfo, std::string *_callName=NULL, int _ignoreIAdjacentMultibodies=1);
};

class PQP_Solid : public PQP {
 public:
  PQP_Solid() : PQP() {m_name = "PQP_SOLID";}
  virtual ~PQP_Solid() {}
  
  virtual bool IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle,
			     StatClass& _stats, CDInfo& _cdInfo,std::string *_callName=NULL, int ignoreIAdjacentMultibodies=1);
  
  virtual bool IsInsideObstacle(const Cfg& _cfg, Environment* _env);
  virtual bool IsInsideObstacle(Vector3D _robot_pt, shared_ptr<MultiBody> _obstacle);
  
  PQP_Model* BuildPQPSegment(PQP_REAL _dX, PQP_REAL _dY, PQP_REAL _dZ) const;
};
#endif
  
#endif
