#ifndef SOLIDCOLLISIONDETECTION_H
#define SOLIDCOLLISIONDETECTION_H

#ifdef USE_SOLID
#include "CollisionDetectionMethod.h"
#include <SOLID.h>

class Solid : public CollisionDetectionMethod {
 public:
  Solid();
  virtual ~Solid();

  virtual bool IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, 
                             StatClass& _stats, CDInfo& _cdInfo,std::string *_callName=NULL, int _ignoreIAdjacentMultibodies=1);

};
#endif

#endif
